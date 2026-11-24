
/* Registro de mediciones en la flash (SPIFFS). Ver registro.h.
   Orden: constantes, prototipos, estado, funciones publicas, funciones internas. */
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "configuracion.h"
#include "registro.h"
#ifndef REGISTRO_PC
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_spiffs.h"
#endif

/* ===== Constantes ===== */
#ifndef REGISTRO_ARCHIVO
#define REGISTRO_ARCHIVO "/spiffs/mediciones.bin"
#endif
#define REGISTRO_BASE      "/spiffs"
#define REGISTRO_PARTICION "storage"
#define CSV_BOM            "\xEF\xBB\xBF"   /* para que Excel lea los acentos */
#define CSV_ENCABEZADO     "N\xC2\xBA de medici\xC3\xB3n;Fecha;Hora;RPM;Estado;Observaciones\r\n"

/* ===== Prototipos ===== */
static void bloquear(void);
static void desbloquear(void);
static esp_err_t escribir(FILE *f,unsigned indice,const registro_t *r);
static bool leer(unsigned indice,registro_t *r);
static void fecha_hora(const registro_t *r,char *fecha,size_t nf,char *hora,size_t nh);
static void observaciones(const registro_t *r,char *b,size_t n);
static bool caracter_valido(unsigned char c);
static size_t largo_utf8(unsigned char c);

/* ===== Estado ===== */
#ifndef REGISTRO_PC
static SemaphoreHandle_t mutex;
#endif
static unsigned cantidad;
static bool disponible;

/* ===== Funciones publicas ===== */

/* Monta la particion y cuenta los registros guardados. */
esp_err_t registro_iniciar(void) {
#ifndef REGISTRO_PC
    mutex=xSemaphoreCreateMutex();
    if(!mutex) return ESP_ERR_NO_MEM;
    esp_vfs_spiffs_conf_t conf={.base_path=REGISTRO_BASE,
        .partition_label=REGISTRO_PARTICION,.max_files=3,
        .format_if_mount_failed=true};
    esp_err_t err=esp_vfs_spiffs_register(&conf);
    if(err!=ESP_OK) return err;
#endif
    cantidad=0;
    FILE *f=fopen(REGISTRO_ARCHIVO,"rb");
    if(f) {
        /* Un registro incompleto al final (corte de energia) se ignora
           y se sobreescribe con el siguiente. */
        if(fseek(f,0,SEEK_END)==0) {
            long t=ftell(f);
            if(t>0) cantidad=(unsigned)(t/(long)sizeof(registro_t));
        }
        fclose(f);
    }
    disponible=true;
    return ESP_OK;
}

unsigned registro_cantidad(void) {
    return cantidad;
}

/* Agrega un registro al final y le asigna el numero de medicion. */
esp_err_t registro_agregar(registro_t *r) {
    if(!disponible) return ESP_ERR_INVALID_STATE;
    esp_err_t err;
    bloquear();
    if(cantidad>=REGISTRO_MAXIMO) err=ESP_ERR_NO_MEM;
    else {
        r->numero=cantidad+1;
        r->obs[REGISTRO_OBS_MAX-1]=0;
        FILE *f=fopen(REGISTRO_ARCHIVO,"r+b");
        if(!f) f=fopen(REGISTRO_ARCHIVO,"wb");
        if(!f) err=ESP_FAIL;
        else {
            err=escribir(f,cantidad,r);
            if(fclose(f)!=0) err=ESP_FAIL;
        }
        if(err==ESP_OK) cantidad++;
    }
    desbloquear();
    return err;
}

/* Reemplaza el texto de observacion de una medicion. */
esp_err_t registro_observacion(uint32_t numero,const char *texto) {
    if(!disponible) return ESP_ERR_INVALID_STATE;
    if(numero<1 || numero>cantidad) return ESP_ERR_NOT_FOUND;
    registro_t r;
    if(!leer(numero-1,&r)) return ESP_FAIL;
    registro_limpiar_texto(r.obs,texto,sizeof r.obs);
    esp_err_t err=ESP_FAIL;
    bloquear();
    FILE *f=fopen(REGISTRO_ARCHIVO,"r+b");
    if(f) {
        err=escribir(f,numero-1,&r);
        if(fclose(f)!=0) err=ESP_FAIL;
    }
    desbloquear();
    return err;
}

/* Completa la hora de las mediciones hechas en este arranque antes de
   conocer la hora real, usando el tiempo transcurrido desde el arranque. */
esp_err_t registro_completar_hora(uint16_t arranque,uint32_t epoch_ahora,
                                  uint32_t uptime_ahora,int16_t zona_min) {
    if(!disponible) return ESP_ERR_INVALID_STATE;
    esp_err_t err=ESP_OK;
    bloquear();
    FILE *f=fopen(REGISTRO_ARCHIVO,"r+b");
    if(f) {
        registro_t r;
        for(unsigned i=0;i<cantidad && err==ESP_OK;i++) {
            if(fseek(f,(long)i*(long)sizeof r,SEEK_SET)!=0 ||
               fread(&r,sizeof r,1,f)!=1) {err=ESP_FAIL; break;}
            if(r.epoch || r.arranque!=arranque || r.uptime_s>uptime_ahora) continue;
            r.epoch=epoch_ahora-(uptime_ahora-r.uptime_s);
            r.zona_min=zona_min;
            err=escribir(f,i,&r);
        }
        if(fclose(f)!=0) err=ESP_FAIL;
    }
    desbloquear();
    return err;
}

/* Borra todas las mediciones. La numeracion vuelve a empezar en 1. */
esp_err_t registro_borrar(void) {
    if(!disponible) return ESP_ERR_INVALID_STATE;
    bloquear();
    FILE *f=fopen(REGISTRO_ARCHIVO,"wb");
    esp_err_t err=f && fclose(f)==0 ? ESP_OK : ESP_FAIL;
    remove(REGISTRO_ARCHIVO);
    if(err==ESP_OK) cantidad=0;
    desbloquear();
    return err;
}

/* Planilla separada por ';' (formato de Excel en espanol). */
esp_err_t registro_csv(registro_salida_t salida,void *ctx,bool bom) {
    if(!disponible) return ESP_ERR_INVALID_STATE;
    esp_err_t err=ESP_OK;
    if(bom) err=salida(ctx,CSV_BOM,strlen(CSV_BOM));
    if(err==ESP_OK) err=salida(ctx,CSV_ENCABEZADO,strlen(CSV_ENCABEZADO));
    registro_t r;
    char fecha[12],hora[10],obs[96],linea[200];
    for(unsigned i=0;i<cantidad && err==ESP_OK;i++) {
        if(!leer(i,&r)) {err=ESP_FAIL; break;}
        fecha_hora(&r,fecha,sizeof fecha,hora,sizeof hora);
        observaciones(&r,obs,sizeof obs);
        int n=snprintf(linea,sizeof linea,"%u;%s;%s;%.0f;%s;%s\r\n",
            (unsigned)r.numero,fecha,hora,r.rpm,
            r.calibrada?"ESTABLE":"SIN CALIBRAR",obs);
        if(n<0) {err=ESP_FAIL; break;}
        if((size_t)n>=sizeof linea) n=(int)sizeof linea-1;
        err=salida(ctx,linea,(size_t)n);
    }
    return err;
}

/* Lista para la app: [{"n":1,"t":epoch,"z":-180,"rpm":1500,"cal":1,
   "ppr":1,"f":1.000,"obs":"..."}, ...]. El texto ya esta limpio. */
esp_err_t registro_json(registro_salida_t salida,void *ctx) {
    if(!disponible) return ESP_ERR_INVALID_STATE;
    esp_err_t err=salida(ctx,"[",1);
    registro_t r;
    char linea[200];
    for(unsigned i=0;i<cantidad && err==ESP_OK;i++) {
        if(!leer(i,&r)) {err=ESP_FAIL; break;}
        int n=snprintf(linea,sizeof linea,
            "%s{\"n\":%u,\"t\":%u,\"z\":%d,\"rpm\":%.0f,\"cal\":%d,"
            "\"ppr\":%u,\"f\":%.3f,\"obs\":\"%s\"}",
            i?",":"",(unsigned)r.numero,(unsigned)r.epoch,(int)r.zona_min,
            r.rpm,r.calibrada?1:0,(unsigned)r.ppr,r.factor,r.obs);
        if(n<0 || (size_t)n>=sizeof linea) {err=ESP_FAIL; break;}
        err=salida(ctx,linea,(size_t)n);
    }
    if(err==ESP_OK) err=salida(ctx,"]",1);
    return err;
}

/* Copia un texto quitando lo que romperia la planilla o la lista
   (comillas, barras, ';', '<', '>', '&' y controles). Respeta los
   caracteres UTF-8 (acentos, ñ) y no los corta a la mitad. */
void registro_limpiar_texto(char *destino,const char *origen,size_t n) {
    if(!n) return;
    size_t j=0;
    const unsigned char *s=(const unsigned char *)origen;
    while(*s) {
        size_t largo=largo_utf8(*s);
        if(largo==1) {
            char c=caracter_valido(*s)?(char)*s:' ';
            s++;
            if(c==' ' && j && destino[j-1]==' ') continue;   /* sin espacios dobles */
            if(j+1>n-1) break;
            destino[j++]=c;
            continue;
        }
        size_t k=1;
        while(k<largo && (s[k]&0xC0)==0x80) k++;
        if(!largo || k<largo) {s+=k?k:1; continue;}   /* UTF-8 invalido */
        if(j+largo>n-1) break;
        memcpy(destino+j,s,largo); j+=largo; s+=largo;
    }
    while(j && destino[j-1]==' ') j--;
    destino[j]=0;
    /* Saltar espacios iniciales. */
    size_t i=0;
    while(destino[i]==' ') i++;
    if(i) memmove(destino,destino+i,j-i+1);
}

/* ===== Funciones internas ===== */

static void bloquear(void) {
#ifndef REGISTRO_PC
    xSemaphoreTake(mutex,portMAX_DELAY);
#endif
}

static void desbloquear(void) {
#ifndef REGISTRO_PC
    xSemaphoreGive(mutex);
#endif
}

static esp_err_t escribir(FILE *f,unsigned indice,const registro_t *r) {
    if(fseek(f,(long)indice*(long)sizeof *r,SEEK_SET)!=0) return ESP_FAIL;
    if(fwrite(r,sizeof *r,1,f)!=1) return ESP_FAIL;
    return fflush(f)==0 ? ESP_OK : ESP_FAIL;
}

/* Lee un registro. Bloquea solo durante la lectura, para no frenar al
   lazo principal mientras se envia una planilla larga. */
static bool leer(unsigned indice,registro_t *r) {
    bool ok=false;
    bloquear();
    FILE *f=fopen(REGISTRO_ARCHIVO,"rb");
    if(f) {
        ok=fseek(f,(long)indice*(long)sizeof *r,SEEK_SET)==0 &&
           fread(r,sizeof *r,1,f)==1;
        fclose(f);
    }
    desbloquear();
    if(ok) r->obs[REGISTRO_OBS_MAX-1]=0;
    return ok;
}

static void fecha_hora(const registro_t *r,char *fecha,size_t nf,char *hora,size_t nh) {
    if(!r->epoch) {
        snprintf(fecha,nf,"sin hora");
        snprintf(hora,nh,"%s","");
        return;
    }
    time_t t=(time_t)r->epoch+(time_t)r->zona_min*60;
    struct tm tm;
    gmtime_r(&t,&tm);
    snprintf(fecha,nf,"%02d/%02d/%04d",tm.tm_mday,tm.tm_mon+1,tm.tm_year+1900);
    snprintf(hora,nh,"%02d:%02d:%02d",tm.tm_hour,tm.tm_min,tm.tm_sec);
}

/* Datos automaticos de la medicion mas el texto del usuario. */
static void observaciones(const registro_t *r,char *b,size_t n) {
    char factor[16];
    snprintf(factor,sizeof factor,"%.3f",r->factor);
    char *punto=strchr(factor,'.');
    if(punto) *punto=',';   /* coma decimal */
    snprintf(b,n,"PPR %u, factor %s%s%s",(unsigned)r->ppr,factor,
             r->obs[0]?" - ":"",r->obs);
}

static bool caracter_valido(unsigned char c) {
    return c>=0x20 && c<0x7f && !strchr("\"\\;<>&",c);
}

/* Bytes que ocupa el caracter UTF-8 que empieza con c (0 = invalido). */
static size_t largo_utf8(unsigned char c) {
    if(c<0x80) return 1;
    if((c&0xE0)==0xC0) return 2;
    if((c&0xF0)==0xE0) return 3;
    if((c&0xF8)==0xF0) return 4;
    return 0;
}


