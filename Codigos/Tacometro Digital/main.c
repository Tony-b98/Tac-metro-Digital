
/* Tacometro real ESP8266, C, ESP8266_RTOS_SDK v3.4. Ver CONEXIONES.md.
   Orden: constantes, tipos, prototipos, estado, app_main, implementaciones. */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/pwm.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "configuracion.h"
#include "captura.h"
#include "registro.h"
#include "web.h"

/* ===== Constantes ===== */
#define GRACIA_US           3000000LL  /* margen antes de avisar error o falta de calibracion */
#define ESTABLE_VENTANAS    3U
#define ESTABLE_ANTES_SALTO 2U
#define ESTABLE_TIEMPO_US   1500000LL
#define ESTABLE_TOLERANCIA  0.05f
#define PANTALLA_US         250000LL
#define ALTERNAR_US         2000000LL  /* el LCD alterna mensajes cada 2 s */
#define PITIDO_US           200000LL
#define ALARMA_CICLO_US     500000LL
#define ALARMA_SONIDO_US    200000LL
#define BOTON_REBOTE_US     40000LL
#define BOTON_LARGO_US      3000000LL
#define REF_MIN_RPM         60.0f
#define REF_MAX_RPM         12000.0f
#define FACTOR_MIN          0.5f
#define FACTOR_MAX          2.0f
#define PPR_MAX             20U
#define BUZZER_PERIODO_US   500U       /* 2 kHz */
#define BUZZER_DUTY         250U       /* 50 % */
#define PITIDOS_INICIO      1U
#define PITIDOS_CAL         2U
#define PITIDOS_FIN         3U

/* ===== Tipos ===== */
typedef struct {int pin; int previo,firme; int64_t cambio;} boton_t;

/* ===== Prototipos ===== */
/* Sensor y medicion */
static void pulso(void *arg);
static inline uint32_t filtro_sensor(void);
static void reset_captura(void);
static void medir(void);
static void pausar_sensor(void);
static void reanudar_sensor(void);
/* LCD 1602 por PCF8574 */
static void esperar_us(unsigned us);
static esp_err_t i2c_escribir(uint8_t direccion,uint8_t *datos,size_t n);
static void lcd_nibble(uint8_t x);
static void lcd_byte(uint8_t x,bool dato);
static void lcd_init(void);
static void linea(unsigned fila,const char *texto);
/* Configuracion persistente */
static void cargar_configuracion(void);
static esp_err_t guardar(void);
static const char *aplicar_referencia(float ref);
static void guardar_red(void);
/* Registro de mediciones y app web */
static void guardar_medicion(void);
static void atender_web(int64_t ahora);
static void publicar_estado(void);
static esp_err_t salida_serie(void *ctx,const char *texto,size_t n);
/* Control */
static void pitidos(unsigned cantidad);
static void iniciar(void);
static void detener(void);
static void alternar_calibracion(void);
static void restaurar(void);
static void comando(const char *s);
/* Hardware y lazo principal */
static void configurar_hardware(void);
static bool boton(boton_t *b);
static void atender_botones(int64_t ahora);
static void leer_serie(void);
static void actualizar_indicadores(int64_t ahora);
static bool rpm_mostrada(float *valor);
static const char *texto_estado(void);
static void actualizar_pantalla(int64_t ahora);

/* ===== Estado ===== */
/* Captura compartida con la ISR. */
static volatile captura_t captura;
static volatile bool marcha;
/* Filtro del sensor: nivel crudo, ultimo cambio y nivel confirmado.
   Cada nivel se confirma solo si duro lo suficiente: una marca genera un
   unico evento aunque sus flancos tengan rebotes. */
static volatile int sensor_nivel, sensor_firme;
static volatile int64_t sensor_cambio;

static bool calibrando, estable, senal, rango;
static int64_t ventana_desde;
static float estabilidad_base;
static unsigned ventanas_estables, pulsos_ventana, descartados_ventana;
static bool irregular, salto_ignorado;
static float rpm, cruda, factor=1;
static float ultima_rpm, ultima_estable_rpm;
static bool hubo_lectura_estable;
static bool lectura_guardada;
/* Resultado final retenido en RAM hasta INICIO o reinicio del equipo. */
static bool medida_finalizada;
static unsigned ppr=1;
static bool calibrada, respaldo;
static float factor_ref=1;
static unsigned ppr_ref=1;
static int64_t desde_estable, inicio_medicion;
static nvs_handle memoria;
static float persist_factor=1, persist_ref=1;
static unsigned persist_ppr=1, persist_refppr=1;
static bool persist_cal, persist_respaldo;
static unsigned pitido_fases;
static int64_t pitido_hasta;

/* Registro de mediciones */
static uint16_t arranque;            /* cuenta de arranques, para completar horas */
static bool registro_listo;
static bool hay_registro;            /* la ultima medicion intento guardarse */
static esp_err_t registro_estado;    /* resultado de ese intento */
static uint32_t ultimo_numero;
static bool hora_valida_previa;

/* WiFi (hotspot del celular) */
static char red_ssid[33], red_clave[65];
static int64_t proxima_publicacion;

/* LCD */
static uint8_t lcd_addr=LCD_ADDR;
static bool lcd_disponible;

/* Lazo principal */
static boton_t boton_inicio={INICIO,1,1,0}, boton_cal={CAL,1,1,0};
static int64_t cal_desde, proxima_pantalla, alarma_desde;
static bool larga, alarma_previa, sonido_anterior;
static char entrada[80];
static unsigned entrada_len;

/* ===== Programa principal ===== */
void app_main(void) {
    cargar_configuracion();
    configurar_hardware();
    lcd_init();
    esp_err_t err=registro_iniciar();
    registro_listo=err==ESP_OK;
    if(registro_listo) printf("Mediciones guardadas: %u\n",registro_cantidad());
    else printf("Memoria de mediciones no disponible: %s. Revisar partitions.csv\n",
                esp_err_to_name(err));
    web_iniciar(red_ssid,red_clave);
    printf("Tacometro ESP8266 v7 LCD1602 registro de mediciones y app web listo. Usar referencia real para calibrar.\n");
    printf("INICIO, PARAR, CAL, REF <rpm>, PPR <1..20>, RESTAURAR, REGISTROS, BORRAR TODO, RED, SSID <red>, CLAVE <clave>\n");
    for(;;) {
        int64_t ahora=esp_timer_get_time();
        medir();
        atender_botones(ahora);
        leer_serie();
        atender_web(ahora);
        actualizar_indicadores(ahora);
        actualizar_pantalla(ahora);
        vTaskDelay(1);
    }
}

/* ===== Sensor y medicion ===== */

/* Filtro proporcional al ultimo periodo medido. Sin periodo: el minimo. */
static inline IRAM_ATTR uint32_t filtro_sensor(void) {
    uint32_t f=captura.periodo_us/SENSOR_FILTRO_DIV;
    if(f<SENSOR_FILTRO_MIN_US) f=SENSOR_FILTRO_MIN_US;
    if(f>SENSOR_FILTRO_MAX_US) f=SENSOR_FILTRO_MAX_US;
    return f;
}

/* ISR del sensor (ambos flancos). El nivel que termina se confirma si duro
   al menos el filtro; un rebote corto no cambia el nivel confirmado.
   Marca = nivel bajo; se mide entre sus flancos de entrada. */
static void IRAM_ATTR pulso(void *arg) {
    (void)arg;
    int nivel=gpio_get_level(SENSOR);
    int64_t ahora=esp_timer_get_time();
    if(nivel==sensor_nivel) return;
    int64_t duracion=ahora-sensor_cambio;
    if(sensor_nivel!=sensor_firme && duracion>=(int64_t)filtro_sensor()) {
        sensor_firme=sensor_nivel;
        if(!sensor_firme && marcha)
            captura_evento(&captura,sensor_cambio,ppr);
    }
    sensor_nivel=nivel; sensor_cambio=ahora;
}

static void reset_captura(void) {
    portENTER_CRITICAL();
    captura_reset(&captura);
    sensor_nivel=gpio_get_level(SENSOR);
    sensor_firme=sensor_nivel;
    sensor_cambio=esp_timer_get_time();
    portEXIT_CRITICAL();
    rpm=cruda=0; senal=estable=rango=irregular=salto_ignorado=false;
    ventanas_estables=0; estabilidad_base=0;
    pulsos_ventana=descartados_ventana=0;
    ventana_desde=esp_timer_get_time(); desde_estable=0;
}

static void medir(void) {
    if(!marcha) return;
    int64_t ahora=esp_timer_get_time(),last;
    captura_t muestra={0};
    bool nueva=false;
    portENTER_CRITICAL();
    last=captura.ultimo;
    if(last && ahora-last>=CAPTURA_TIMEOUT_US) {
        /* Sin marcas: reiniciar tambien el filtro adaptativo. */
        captura_reset(&captura); last=0;
    }
    if(ahora-ventana_desde>=CAPTURA_VENTANA_US) {
        muestra=captura; captura_limpiar(&captura); nueva=true;
    }
    portEXIT_CRITICAL();
    if(!last) {
        rpm=cruda=0; senal=rango=estable=irregular=salto_ignorado=false;
        ventanas_estables=0; desde_estable=0;
    }
    if(!nueva) return;
    ventana_desde=ahora;
    pulsos_ventana=muestra.periodos; descartados_ventana=muestra.descartados;
    if(!muestra.periodos) return;
    resultado_t r=captura_resultado(&muestra,ppr,factor);
    bool candidato=r.rango && r.regular;
    /* Una ventana anómala no aporta RPM ni cuenta para estabilizar.
       Mantener la última lectura hasta recibir otra ventana válida.
       Si la anomalía se repite, se informa y se reinicia la estabilidad. */
    if(!candidato && ventanas_estables>=ESTABLE_ANTES_SALTO && !salto_ignorado) {
        salto_ignorado=true;
        estable=false;
        return;
    }
    salto_ignorado=false;
    senal=true; rango=r.rango; irregular=!r.regular;
    cruda=r.cruda; rpm=r.rango?r.rpm:0;
    if(rango && !irregular) {ultima_rpm=rpm; lectura_guardada=true;}
    if(!candidato) {ventanas_estables=0; desde_estable=0;}
    else if(!ventanas_estables ||
            fabsf(cruda-estabilidad_base)>estabilidad_base*ESTABLE_TOLERANCIA) {
        estabilidad_base=cruda; ventanas_estables=1; desde_estable=ahora;
    } else if(ventanas_estables<ESTABLE_VENTANAS) ventanas_estables++;
    estable=candidato && ventanas_estables>=ESTABLE_VENTANAS &&
            ahora-desde_estable>=ESTABLE_TIEMPO_US;
    if(estable && !calibrando) {
        ultima_estable_rpm=rpm;
        hubo_lectura_estable=true;
    }
}

/* Sin interrupcion del sensor mientras se escribe la flash. */
static void pausar_sensor(void) {
    ESP_ERROR_CHECK(gpio_set_intr_type(SENSOR,GPIO_INTR_DISABLE));
}

static void reanudar_sensor(void) {
    ESP_ERROR_CHECK(gpio_set_intr_type(SENSOR,GPIO_INTR_ANYEDGE));
}

/* ===== LCD 1602 =====
   PCF8574 del LCD: RS=P0, RW=P1, E=P2, luz=P3, datos=P4..P7. */

static void esperar_us(unsigned us) {
    int64_t desde=esp_timer_get_time();
    while(esp_timer_get_time()-desde<us) { }
}

static esp_err_t i2c_escribir(uint8_t direccion,uint8_t *datos,size_t n) {
    i2c_cmd_handle_t cmd=i2c_cmd_link_create();
    if(!cmd) return ESP_ERR_NO_MEM;
    esp_err_t err=i2c_master_start(cmd);
    if(err==ESP_OK) err=i2c_master_write_byte(cmd,(direccion<<1)|I2C_MASTER_WRITE,true);
    if(err==ESP_OK && n) err=i2c_master_write(cmd,datos,n,true);
    if(err==ESP_OK) err=i2c_master_stop(cmd);
    if(err==ESP_OK) err=i2c_master_cmd_begin(I2C_NUM_0,cmd,pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}

static void lcd_nibble(uint8_t x) {
    if(!lcd_disponible) return;
    uint8_t b[]={x|8,x|12,x|8};
    if(i2c_escribir(lcd_addr,b,sizeof b)!=ESP_OK) {
        lcd_disponible=false;
        printf("LCD desconectado. Revisar cableado y reiniciar.\n");
        return;
    }
    esperar_us(60);
}

static void lcd_byte(uint8_t x,bool dato) {
    lcd_nibble((x&0xf0)|(dato?1:0));
    lcd_nibble((x<<4)|(dato?1:0));
}

static void lcd_init(void) {
    /* En ESP8266 se instala I2C antes de configurarlo. */
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0,I2C_MODE_MASTER));
    i2c_config_t c={.mode=I2C_MODE_MASTER,.sda_io_num=LCD_SDA,
        .scl_io_num=LCD_SCL,.sda_pullup_en=GPIO_PULLUP_ENABLE,
        .scl_pullup_en=GPIO_PULLUP_ENABLE,.clk_stretch_tick=300};
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0,&c));
    esp_err_t err=i2c_escribir(lcd_addr,NULL,0);
    if(err!=ESP_OK) {
        uint8_t alternativa=lcd_addr==0x27?0x3f:0x27;
        err=i2c_escribir(alternativa,NULL,0);
        if(err==ESP_OK) lcd_addr=alternativa;
    }
    if(err!=ESP_OK) {
        /* Rangos del PCF8574 (0x20..0x27) y PCF8574A (0x38..0x3f). */
        for(unsigned a=0x20;a<=0x3f;a++) {
            if(a>0x27 && a<0x38) continue;
            if(i2c_escribir(a,NULL,0)==ESP_OK) {lcd_addr=a;err=ESP_OK;break;}
        }
    }
    if(err!=ESP_OK) {
        printf("LCD no responde. Revisar D2/SDA, D1/SCL, convertidor y GND.\n");
        return;
    }
    lcd_disponible=true;
    printf("LCD I2C: 0x%02x\n",lcd_addr);
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_nibble(0x30); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_nibble(0x30); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_nibble(0x30); lcd_nibble(0x20);
    lcd_byte(0x28,false); lcd_byte(0x0c,false); lcd_byte(0x06,false);
    lcd_byte(0x01,false); vTaskDelay(pdMS_TO_TICKS(3));
}

static void linea(unsigned fila,const char *texto) {
    if(!lcd_disponible || fila>=2) return;
    static const uint8_t offset[]={0,0x40};
    lcd_byte(0x80|offset[fila],false);
    size_t n=strlen(texto);
    for(unsigned i=0;i<16;i++) lcd_byte(i<n?texto[i]:' ',true);
}

/* ===== Configuracion persistente ===== */

static void cargar_configuracion(void) {
    esp_err_t err=nvs_flash_init();
    if(err==ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase()); err=nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(nvs_open("tacometro",NVS_READWRITE,&memoria));
    size_t sz=sizeof factor;
    nvs_get_blob(memoria,"factor",&factor,&sz);
    uint32_t p=1; nvs_get_u32(memoria,"ppr",&p); ppr=p;
    uint8_t c=0; nvs_get_u8(memoria,"cal",&c); calibrada=c==1;
    if(!isfinite(factor) || factor<FACTOR_MIN || factor>FACTOR_MAX ||
       ppr<1 || ppr>PPR_MAX) {
        factor=1; ppr=1; calibrada=false;
    }
    sz=sizeof factor_ref;
    respaldo=nvs_get_blob(memoria,"ref",&factor_ref,&sz)==ESP_OK;
    p=1; respaldo=respaldo && nvs_get_u32(memoria,"refppr",&p)==ESP_OK;
    ppr_ref=p;
    respaldo=respaldo && isfinite(factor_ref) &&
        factor_ref>=FACTOR_MIN && factor_ref<=FACTOR_MAX && p>=1 && p<=PPR_MAX;
    persist_factor=factor; persist_ppr=ppr; persist_cal=calibrada;
    persist_ref=factor_ref; persist_refppr=ppr_ref; persist_respaldo=respaldo;
    /* Red WiFi guardada o la de configuracion.h. */
    sz=sizeof red_ssid;
    if(nvs_get_str(memoria,"ssid",red_ssid,&sz)!=ESP_OK)
        snprintf(red_ssid,sizeof red_ssid,"%s",WIFI_SSID);
    sz=sizeof red_clave;
    if(nvs_get_str(memoria,"clave",red_clave,&sz)!=ESP_OK)
        snprintf(red_clave,sizeof red_clave,"%s",WIFI_CLAVE);
    /* Contar arranques: las mediciones sin hora de este arranque se
       completan cuando llega la hora del celular o de internet. */
    uint16_t a=0;
    nvs_get_u16(memoria,"arranques",&a);
    arranque=a+1;
    if(!arranque) arranque=1;
    if(nvs_set_u16(memoria,"arranques",arranque)==ESP_OK) nvs_commit(memoria);
}

static esp_err_t guardar(void) {
    pausar_sensor();
    esp_err_t err=nvs_set_blob(memoria,"factor",&factor,sizeof factor);
    if(err==ESP_OK) err=nvs_set_u32(memoria,"ppr",ppr);
    if(err==ESP_OK) err=nvs_set_u8(memoria,"cal",calibrada);
    if(err==ESP_OK && respaldo) err=nvs_set_blob(memoria,"ref",&factor_ref,sizeof factor_ref);
    if(err==ESP_OK && respaldo) err=nvs_set_u32(memoria,"refppr",ppr_ref);
    if(err==ESP_OK) err=nvs_commit(memoria);
    printf("Guardado: %s\n",esp_err_to_name(err));
    if(err==ESP_OK) {
        persist_factor=factor; persist_ppr=ppr; persist_cal=calibrada;
        persist_ref=factor_ref; persist_refppr=ppr_ref; persist_respaldo=respaldo;
    } else {
        factor=persist_factor; ppr=persist_ppr; calibrada=persist_cal;
        factor_ref=persist_ref; ppr_ref=persist_refppr; respaldo=persist_respaldo;
        /* Conservar configuracion anterior en RAM y reabrir el namespace. */
        nvs_close(memoria);
        ESP_ERROR_CHECK(nvs_open("tacometro",NVS_READWRITE,&memoria));
    }
    reset_captura();
    reanudar_sensor();
    inicio_medicion=esp_timer_get_time();
    return err;
}

/* Guarda la red WiFi y se reconecta. Solo con la medicion detenida. */
static void guardar_red(void) {
    pausar_sensor();
    esp_err_t err=nvs_set_str(memoria,"ssid",red_ssid);
    if(err==ESP_OK) err=nvs_set_str(memoria,"clave",red_clave);
    if(err==ESP_OK) err=nvs_commit(memoria);
    reanudar_sensor();
    printf("Red guardada: %s\n",esp_err_to_name(err));
    web_cambiar_red(red_ssid,red_clave);
}

/* ===== Registro de mediciones y app web ===== */

/* Guarda la medicion finalizada. Si todavia no hay hora, se completa
   despues, cuando el celular o internet la envien (mismo arranque). */
static void guardar_medicion(void) {
    hay_registro=true;
    if(!registro_listo) {registro_estado=ESP_ERR_INVALID_STATE; return;}
    registro_t r;
    memset(&r,0,sizeof r);
    r.uptime_s=(uint32_t)(esp_timer_get_time()/1000000);
    r.arranque=arranque;
    r.zona_min=web_zona_min();
    r.epoch=web_hora_valida()?(uint32_t)time(NULL):0;
    r.ppr=(uint16_t)ppr;
    r.calibrada=calibrada;
    r.rpm=ultima_rpm;
    r.factor=factor;
    pausar_sensor();
    registro_estado=registro_agregar(&r);
    reanudar_sensor();
    if(registro_estado==ESP_OK) {
        ultimo_numero=r.numero;
        printf("Medicion N %u guardada%s\n",(unsigned)r.numero,
               r.epoch?"":" (sin hora: se completa al abrir la app del celular)");
    } else if(registro_estado==ESP_ERR_NO_MEM) {
        printf("Memoria de mediciones llena: descargar la planilla y borrar.\n");
    } else {
        printf("Error al guardar la medicion: %s\n",esp_err_to_name(registro_estado));
    }
}

/* Publica el estado para la app y, con la medicion detenida, aplica
   lo que pidio la app (la flash no se toca mientras se mide). */
static void atender_web(int64_t ahora) {
    if(ahora>=proxima_publicacion) {
        publicar_estado();
        proxima_publicacion=ahora+PANTALLA_US;
    }
    web_pedido_t p;
    if(web_tomar_pedido(&p)) {
        const char *error=NULL;
        if(p.cal) {
            if(marcha && !calibrando) error="Detené la medición antes de calibrar.";
            else if(!calibrando) alternar_calibracion();
        } else if(p.referencia) error=aplicar_referencia(p.ref);
        else if(marcha || !registro_listo) error="Historial no disponible durante la medición.";
        else {
            pausar_sensor();
            esp_err_t err=p.borrar?registro_borrar():registro_observacion(p.numero,p.texto);
            reanudar_sensor();
            if(err!=ESP_OK) error="No se pudo actualizar el historial.";
            else if(p.borrar) hay_registro=false;
        }
        publicar_estado();
        web_responder(error);
    }
    if(marcha || !registro_listo) return;
    bool hora_valida=web_hora_valida();
    if(hora_valida && !hora_valida_previa) {
        pausar_sensor();
        esp_err_t err=registro_completar_hora(arranque,(uint32_t)time(NULL),
            (uint32_t)(esp_timer_get_time()/1000000),web_zona_min());
        reanudar_sensor();
        printf("Hora sincronizada%s\n",err==ESP_OK?"":", error al completar mediciones");
    }
    hora_valida_previa=hora_valida;
}

static void publicar_estado(void) {
    web_estado_t e;
    memset(&e,0,sizeof e);
    e.rpm_valida=rpm_mostrada(&e.rpm);
    snprintf(e.estado,sizeof e.estado,"%s",texto_estado());
    e.marcha=marcha;
    e.calibrando=calibrando;
    e.calibrada=calibrada;
    e.finalizada=medida_finalizada;
    e.ppr=ppr;
    e.factor=factor;
    e.registros=registro_listo?registro_cantidad():0;
    web_publicar(&e);
}

static esp_err_t salida_serie(void *ctx,const char *texto,size_t n) {
    (void)ctx;
    fwrite(texto,1,n,stdout);
    return ESP_OK;
}

/* ===== Control ===== */

static void pitidos(unsigned cantidad) {
    if(!cantidad) return;
    pitido_fases=cantidad*2-1;
    pitido_hasta=esp_timer_get_time()+PITIDO_US;
}

static void iniciar(void) {
    marcha=true; calibrando=false; reset_captura();
    ultima_rpm=0; lectura_guardada=false; medida_finalizada=false;
    hubo_lectura_estable=false;
    hay_registro=false;
    inicio_medicion=esp_timer_get_time();
    pitidos(PITIDOS_INICIO);
    printf("Medicion iniciada\n");
}

static void detener(void) {
    marcha=calibrando=false;
    pitido_fases=0;
    reset_captura();
    printf("Medicion detenida. Ultima lectura: %.0f RPM\n",ultima_rpm);
}

static void alternar_calibracion(void) {
    if(calibrando) {
        detener();
        return;
    }
    if(marcha) {
        printf("Detener la medicion antes de calibrar\n");
        return;
    }
    iniciar();
    calibrando=true;
    pitidos(PITIDOS_CAL);
}

static void restaurar(void) {
    if(marcha && !calibrando) return;
    if(!respaldo) {printf("No hay referencia guardada\n"); return;}
    factor=factor_ref; ppr=ppr_ref; calibrada=true;
    if(guardar()==ESP_OK && calibrando) detener();
}

static const char *aplicar_referencia(float ref) {
    float f=cruda>0?ref/cruda:0;
    if(!calibrando || !estable || !isfinite(ref) ||
       ref<REF_MIN_RPM || ref>REF_MAX_RPM || f<FACTOR_MIN || f>FACTOR_MAX)
        return "Activá la calibración, esperá una lectura estable y usá una referencia real.";
    factor=f; calibrada=respaldo=true; factor_ref=f; ppr_ref=ppr;
    if(guardar()!=ESP_OK) return "No se pudo guardar la calibración.";
    detener();
    return NULL;
}

static void comando(const char *s) {
    if(!strcmp(s,"INICIO")) iniciar();
    else if(!strcmp(s,"PARAR")) detener();
    else if(!strcmp(s,"BOTONES")) {
        printf("D6_INICIO=%d D7_CAL=%d MARCHA=%u CALIBRANDO=%u\n",
               gpio_get_level(INICIO),gpio_get_level(CAL),
               (unsigned)marcha,(unsigned)calibrando);
    }
    else if(!strcmp(s,"ESTADO")) {
        printf("CAL_GUARDADA=%u FACTOR=%.5f PPR=%u REGISTROS=%u\n",
               (unsigned)(persist_cal && persist_respaldo),persist_factor,
               persist_ppr,registro_listo?registro_cantidad():0);
    }
    else if(!strcmp(s,"CAL")) alternar_calibracion();
    else if(!strcmp(s,"RESTAURAR")) restaurar();
    else if(!strncmp(s,"PPR ",4)) {
        char *fin; long p=strtol(s+4,&fin,10);
        if(marcha || *fin || p<1 || p>(long)PPR_MAX) printf("Detener y usar PPR 1..20\n");
        else {ppr=p; factor=1; calibrada=false; guardar();}
    } else if(!strncmp(s,"REF ",4)) {
        char *fin;
        float ref=strtof(s+4,&fin);
        const char *error=*fin?"RPM de referencia inválidas.":aplicar_referencia(ref);
        if(error) printf("%s\n",error);
    } else if(!strcmp(s,"REGISTROS")) {
        /* Planilla por el cable USB: copiar desde el monitor serie. */
        if(marcha || !registro_listo) printf("Detener la medicion para listar\n");
        else {registro_csv(salida_serie,NULL,false); printf("Fin de la planilla\n");}
    } else if(!strcmp(s,"BORRAR TODO")) {
        if(marcha || !registro_listo) printf("Detener la medicion para borrar\n");
        else {
            pausar_sensor();
            esp_err_t err=registro_borrar();
            reanudar_sensor();
            hay_registro=false;
            printf("Mediciones borradas: %s\n",esp_err_to_name(err));
        }
    } else if(!strcmp(s,"RED")) {
        char ip[16];
        if(web_ip(ip,sizeof ip)) printf("Red \"%s\": conectado, abrir http://%s\n",red_ssid,ip);
        else printf("Red \"%s\": sin conexion\n",red_ssid);
    } else if(!strncmp(s,"SSID ",5) && s[5]) {
        if(marcha) printf("Detener la medicion para cambiar la red\n");
        else {snprintf(red_ssid,sizeof red_ssid,"%s",s+5); guardar_red();}
    } else if(!strcmp(s,"CLAVE") || !strncmp(s,"CLAVE ",6)) {
        if(marcha) printf("Detener la medicion para cambiar la red\n");
        else {snprintf(red_clave,sizeof red_clave,"%s",s[5]?s+6:""); guardar_red();}
    } else printf("Comandos: INICIO PARAR CAL REF <rpm> PPR <1..20> RESTAURAR "
                  "REGISTROS BORRAR TODO RED SSID <red> CLAVE <clave>\n");
}

/* ===== Hardware y lazo principal ===== */

static void configurar_hardware(void) {
    /* Salidas en reposo antes de habilitarlas: LEDs apagados, buzzer mudo. */
    ESP_ERROR_CHECK(gpio_set_level(ROJO,1));
    ESP_ERROR_CHECK(gpio_set_level(VERDE,1));
    ESP_ERROR_CHECK(gpio_set_level(AZUL,0));
    ESP_ERROR_CHECK(gpio_set_level(BUZZER,0));
    gpio_config_t out={.pin_bit_mask=(1U<<ROJO)|(1U<<VERDE)|
        (1U<<AZUL)|(1U<<BUZZER),.mode=GPIO_MODE_OUTPUT,
        .pull_up_en=GPIO_PULLUP_DISABLE,.pull_down_en=GPIO_PULLDOWN_DISABLE,
        .intr_type=GPIO_INTR_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&out));
    uint32_t duty[]={0};
    const uint32_t pwm_gpio[]={BUZZER};
    ESP_ERROR_CHECK(pwm_init(BUZZER_PERIODO_US,duty,1,pwm_gpio));
    ESP_ERROR_CHECK(pwm_set_phase(0,0));
    ESP_ERROR_CHECK(pwm_start());
    gpio_config_t in={.pin_bit_mask=(1U<<SENSOR)|(1U<<INICIO)|(1U<<CAL),
        .mode=GPIO_MODE_INPUT,.pull_up_en=GPIO_PULLUP_ENABLE,
        .pull_down_en=GPIO_PULLDOWN_DISABLE,.intr_type=GPIO_INTR_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&in));
    /* Estado inicial del filtro antes de habilitar la interrupcion. */
    sensor_nivel=gpio_get_level(SENSOR);
    sensor_firme=sensor_nivel;
    sensor_cambio=esp_timer_get_time();
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(SENSOR,pulso,NULL));
    reanudar_sensor();
    uart_config_t uart={.baud_rate=115200,.data_bits=UART_DATA_8_BITS,
        .parity=UART_PARITY_DISABLE,.stop_bits=UART_STOP_BITS_1,
        .flow_ctrl=UART_HW_FLOWCTRL_DISABLE};
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0,&uart));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0,512,0,0,NULL,0));
}

/* Devuelve true cuando el nivel firme cambia tras el antirrebote. */
static bool boton(boton_t *b) {
    int x=gpio_get_level(b->pin); int64_t t=esp_timer_get_time();
    if(x!=b->previo) {b->previo=x; b->cambio=t;}
    if(t-b->cambio>=BOTON_REBOTE_US && x!=b->firme) {b->firme=x; return true;}
    return false;
}

/* INICIO arranca o guarda una medición estable. CAL controla la calibración. */
static void atender_botones(int64_t ahora) {
    if(boton(&boton_inicio) && boton_inicio.firme==0) {
        if(!marcha) iniciar();
        else if(!calibrando) {
            marcha=false;
            medida_finalizada=true;
            lectura_guardada=hubo_lectura_estable;
            if(hubo_lectura_estable) {
                ultima_rpm=ultima_estable_rpm;
                guardar_medicion();
                if(registro_estado==ESP_OK) pitidos(PITIDOS_FIN);
            } else {
                pitido_fases=0;
                printf("Medicion detenida sin una lectura estable para guardar\n");
            }
        }
    }
    if(boton(&boton_cal)) {
        if(boton_cal.firme==0) {cal_desde=ahora; larga=false;}
        else if(!larga && (!calibrada || calibrando)) alternar_calibracion();
    }
    if(boton_cal.firme==0 && !larga && ahora-cal_desde>=BOTON_LARGO_US) {
        larga=true; restaurar();
    }
}

static void leer_serie(void) {
    uint8_t ch;
    while(uart_read_bytes(UART_NUM_0,&ch,1,0)>0) {
        if(ch=='\n' || ch=='\r') {
            if(entrada_len) {entrada[entrada_len]=0; comando(entrada); entrada_len=0;}
        } else if(entrada_len<sizeof entrada-1) entrada[entrada_len++]=ch;
    }
}

static void actualizar_indicadores(int64_t ahora) {
    bool error=marcha && ahora-inicio_medicion>GRACIA_US &&
               (!senal || !rango || irregular);
    /* Una sola indicacion: error, calibracion o estado guardado. */
    bool alarma=marcha && (error || (!calibrada && !calibrando &&
                 ahora-inicio_medicion>=GRACIA_US));
    gpio_set_level(ROJO,!alarma);
    gpio_set_level(VERDE,!((marcha || medida_finalizada) && calibrada &&
                           !calibrando && !error));
    gpio_set_level(AZUL,marcha && calibrando && !error);
    if(alarma && !alarma_previa) {
        alarma_desde=ahora;
        pitido_fases=0;
    }
    alarma_previa=alarma;
    while(pitido_fases && ahora>=pitido_hasta) {
        pitido_fases--;
        pitido_hasta+=PITIDO_US;
    }
    /* Pitidos: 200 ms sonido / 200 ms pausa. Suenan tambien con la medicion
       finalizada (aviso de fin); PARAR los corta.
       Alarma (LED rojo): 200 ms sonido / 300 ms pausa mientras siga. */
    bool sonido=alarma ? (ahora-alarma_desde)%ALARMA_CICLO_US<ALARMA_SONIDO_US :
                         pitido_fases%2!=0;
    if(sonido!=sonido_anterior) {
        ESP_ERROR_CHECK(pwm_set_duty(0,sonido?BUZZER_DUTY:0));
        ESP_ERROR_CHECK(pwm_start());
        sonido_anterior=sonido;
    }
}

/* RPM que se muestran: resultado retenido, lectura valida o ninguna. */
static bool rpm_mostrada(float *valor) {
    if(!marcha && lectura_guardada) {*valor=ultima_rpm; return true;}
    if(senal && rango && !irregular) {*valor=rpm; return true;}
    *valor=0;
    return false;
}

/* Estado para la fila 2 del LCD y para la app. */
static const char *texto_estado(void) {
    if(medida_finalizada) {
        if(!lectura_guardada) return "SIN LECT ESTABLE";
        if(registro_estado==ESP_ERR_NO_MEM) return "MEMORIA LLENA";
        if(registro_estado!=ESP_OK) return "ERROR AL GUARDAR";
        return calibrada?"GUARDADA":"SIN CALIBRAR";
    }
    return
        !marcha?"DETENIDO":calibrando?"CALIBRACION":!senal?
        "SIN SENAL":!rango?"FUERA DE RANGO":irregular?
        "SENAL IRREGULAR":estable?"ESTABLE - INICIO":"ESTABILIZANDO";
}

static void actualizar_pantalla(int64_t ahora) {
    if(ahora<proxima_pantalla) return;
    /* LCD 1602: RPM en fila 1; estado en fila 2.
       El resultado final no cambia aunque el eje se detenga. */
    char b[40];
    float valor;
    if(rpm_mostrada(&valor)) snprintf(b,sizeof b,"RPM: %.0f",valor);
    else snprintf(b,sizeof b,"RPM: --");
    linea(0,b);
    /* Cada 2 s la fila 2 alterna con un dato extra:
       medicion finalizada -> numero guardado; detenido -> IP para el celular. */
    const char *estado=texto_estado();
    char extra[17];
    bool alterna=(ahora/ALTERNAR_US)%2;
    if(alterna && medida_finalizada && hay_registro) {
        if(registro_estado==ESP_OK) snprintf(extra,sizeof extra,"GUARDADA N %u",(unsigned)ultimo_numero);
        else if(registro_estado==ESP_ERR_NO_MEM) snprintf(extra,sizeof extra,"MEMORIA LLENA");
        else snprintf(extra,sizeof extra,"ERROR AL GUARDAR");
        estado=extra;
    } else if(alterna && !marcha && !medida_finalizada) {
        if(!web_ip(extra,sizeof extra)) snprintf(extra,sizeof extra,"SIN WIFI");
        estado=extra;
    }
    linea(1,estado);
    proxima_pantalla=ahora+PANTALLA_US;
    printf("RPM=%.1f senal=%d estable=%d rango=%d pulsos=%u descartados=%u irregular=%d\n",
           rpm,senal,estable,rango,pulsos_ventana,descartados_ventana,irregular);
}



