/* Tacometro real ESP8266, C, ESP8266_RTOS_SDK v3.4. Ver CONEXIONES.md. */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/pwm.h"
#include "configuracion.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "captura.h"
static volatile captura_t captura;
static volatile bool marcha;
/* Confirmar ambos niveles: una marca solo puede generar un evento. */
static volatile int sensor_nivel;
static volatile int64_t sensor_cambio;
static bool sensor_armado;

#define SENSOR_ESTABLE_US 2000LL
static bool calibrando, estable, senal, rango;
static int64_t ventana_desde;
static float estabilidad_base;
static unsigned ventanas_estables, pulsos_ventana, descartados_ventana;
static bool irregular;
static float rpm, cruda, factor=1;
static float ultima_rpm;
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

static void pitidos(unsigned cantidad) {
    pitido_fases=cantidad*2-1;
    pitido_hasta=esp_timer_get_time()+200000;
}
static void reset_captura(void) {
    portENTER_CRITICAL();
    captura_reset(&captura);
    sensor_nivel=gpio_get_level(SENSOR);
    sensor_cambio=esp_timer_get_time();
    sensor_armado=false;
    portEXIT_CRITICAL();
    rpm=cruda=0; senal=estable=rango=irregular=false;
    ventanas_estables=0; estabilidad_base=0;
    pulsos_ventana=descartados_ventana=0;
    ventana_desde=esp_timer_get_time(); desde_estable=0;
}
static void pulso(void *arg) {
    (void)arg;
    int nivel=gpio_get_level(SENSOR);
    int64_t ahora=esp_timer_get_time();
    if(nivel==sensor_nivel) return;
    int64_t duracion=ahora-sensor_cambio;
    if(!nivel) {
        /* Armar solo si previamente hubo salida estable de la marca. */
        sensor_armado=marcha && duracion>=SENSOR_ESTABLE_US;
    } else {
        /* Confirmar la marca completa; medir entre sus flancos de entrada. */
        if(marcha && sensor_armado && duracion>=SENSOR_ESTABLE_US)
            captura_evento(&captura,sensor_cambio,ppr);
        sensor_armado=false;
    }
    sensor_nivel=nivel; sensor_cambio=ahora;
}
static void medir(void) {
    if(!marcha) return;
    int64_t ahora=esp_timer_get_time(),last;
    captura_t muestra={0};
    bool nueva=false;
    portENTER_CRITICAL();
    last=captura.ultimo;
    if(last && ahora-last>=CAPTURA_TIMEOUT_US) {
        captura_reset(&captura); last=0;
    }
    if(ahora-ventana_desde>=CAPTURA_VENTANA_US) {
        muestra=captura; captura_limpiar(&captura); nueva=true;
    }
    portEXIT_CRITICAL();
    if(!last) {
        rpm=cruda=0; senal=rango=estable=irregular=false;
        ventanas_estables=0; desde_estable=0;
    }
    if(!nueva) return;
    ventana_desde=ahora;
    pulsos_ventana=muestra.periodos; descartados_ventana=muestra.descartados;
    if(!muestra.periodos) return;
    resultado_t r=captura_resultado(&muestra,ppr,factor);
    senal=true; rango=r.rango; irregular=!r.regular;
    cruda=r.cruda; rpm=r.rango?r.rpm:0;
    if(rango && !irregular) {ultima_rpm=rpm; lectura_guardada=true;}
    bool candidato=rango && !irregular;
    if(!candidato) {ventanas_estables=0; desde_estable=0;}
    else if(!ventanas_estables || fabsf(cruda-estabilidad_base)>estabilidad_base*0.05f) {
        estabilidad_base=cruda; ventanas_estables=1; desde_estable=ahora;
    } else if(ventanas_estables<3) ventanas_estables++;
    estable=candidato && ventanas_estables>=3 && ahora-desde_estable>=1500000;
    /* Calibrar necesita captura continua para poder recibir REF.
       En medicion normal, retener el primer resultado estable. */
    if(estable && !calibrando) {
        ultima_rpm=rpm; lectura_guardada=true; medida_finalizada=true;
        marcha=false; pitido_fases=0;
        printf("Medicion finalizada: %.1f RPM. Pulsar INICIO para otra.\n",ultima_rpm);
    }
}
/* PCF8574 del LCD: RS=P0, RW=P1, E=P2, luz=P3, datos=P4..P7. */
static uint8_t lcd_addr=LCD_ADDR;
static bool lcd_disponible;
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
static void guardar(void) {
    ESP_ERROR_CHECK(gpio_set_intr_type(SENSOR,GPIO_INTR_DISABLE));
    esp_err_t err=nvs_set_blob(memoria,"factor",&factor,sizeof factor);
    if(err==ESP_OK) err=nvs_set_u32(memoria,"ppr",ppr);
    if(err==ESP_OK) err=nvs_set_u8(memoria,"cal",calibrada);
    if(err==ESP_OK && respaldo) err=nvs_set_blob(memoria,"ref",&factor_ref,sizeof factor_ref);
    if(err==ESP_OK && respaldo) err=nvs_set_u32(memoria,"refppr",ppr_ref);
    if(err==ESP_OK) err=nvs_commit(memoria);
    printf("Guardado: %s\n",esp_err_to_name(err));
    if(err==ESP_OK) {
        persist_factor=factor;persist_ppr=ppr;persist_cal=calibrada;
        persist_ref=factor_ref;persist_refppr=ppr_ref;persist_respaldo=respaldo;
    } else {
        factor=persist_factor;ppr=persist_ppr;calibrada=persist_cal;
        factor_ref=persist_ref;ppr_ref=persist_refppr;respaldo=persist_respaldo;
        /* Conservar configuracion anterior en RAM y reabrir el namespace. */
        nvs_close(memoria);
        ESP_ERROR_CHECK(nvs_open("tacometro",NVS_READWRITE,&memoria));
    }
    reset_captura(); ESP_ERROR_CHECK(gpio_set_intr_type(SENSOR,GPIO_INTR_ANYEDGE));
    inicio_medicion=esp_timer_get_time();
}
static void iniciar(void) {
    marcha=true; calibrando=false; reset_captura();
    ultima_rpm=0; lectura_guardada=false; medida_finalizada=false;
    inicio_medicion=esp_timer_get_time();
    pitidos(1);
    printf("Medicion iniciada\n");
}
static void detener(void) {
    marcha=calibrando=false;
    pitido_fases=0;
    reset_captura();
    printf("Medicion detenida. Ultima lectura: %.0f RPM\n",ultima_rpm);
}
static void alternar_calibracion(void) {
    if(!marcha) iniciar();
    calibrando=!calibrando;
    if(calibrando) pitidos(2);
    else pitido_fases=0;
}
static void restaurar(void) {
    if(!respaldo) {printf("No hay referencia guardada\n");return;}
    factor=factor_ref;ppr=ppr_ref;calibrada=true;calibrando=false;
    guardar();
}
typedef struct {int pin; int previo,firme;int64_t cambio;} boton_t;
static bool boton(boton_t *b) {
    int x=gpio_get_level(b->pin);int64_t t=esp_timer_get_time();
    if(x!=b->previo){b->previo=x;b->cambio=t;}
    if(t-b->cambio>=40000 && x!=b->firme){b->firme=x;return true;}
    return false;
}
static void comando(const char *s) {
    if(!strcmp(s,"INICIO")) iniciar();
    else if(!strcmp(s,"PARAR")) detener();
    else if(!strcmp(s,"CAL")) alternar_calibracion();
    else if(!strcmp(s,"RESTAURAR")) restaurar();
    else if(!strncmp(s,"PPR ",4)) {
        char *fin;long p=strtol(s+4,&fin,10);
        if(marcha || *fin || p<1 || p>20) printf("Detener y usar PPR 1..20\n");
        else {ppr=p;factor=1;calibrada=false;guardar();}
    } else if(!strncmp(s,"REF ",4)) {
        char *fin;float ref=strtof(s+4,&fin);float f=cruda>0?ref/cruda:0;
        if(*fin || !calibrando || !estable || !isfinite(ref) ||
            ref<60 || ref>12000 || f<0.5f || f>2.0f) {
            printf("REF rechazada: activar CAL, estabilizar y usar referencia real\n");
        } else {
            factor=f;calibrada=respaldo=true;factor_ref=f;ppr_ref=ppr;
            calibrando=false;guardar();
        }
    } else printf("Comandos: INICIO PARAR CAL REF <rpm> PPR <1..20> RESTAURAR\n");
}
void app_main(void) {
    esp_err_t err=nvs_flash_init();
    if(err==ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());err=nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(nvs_open("tacometro",NVS_READWRITE,&memoria));
    size_t sz=sizeof factor;
    nvs_get_blob(memoria,"factor",&factor,&sz);
    uint32_t p=1;nvs_get_u32(memoria,"ppr",&p);ppr=p;
    uint8_t c=0;nvs_get_u8(memoria,"cal",&c);calibrada=c==1;
    if(!isfinite(factor)||factor<0.5f||factor>2||ppr<1||ppr>20) {
        factor=1;ppr=1;calibrada=false;
    }
    sz=sizeof factor_ref;
    respaldo=nvs_get_blob(memoria,"ref",&factor_ref,&sz)==ESP_OK;
    p=1;respaldo=respaldo && nvs_get_u32(memoria,"refppr",&p)==ESP_OK;
    ppr_ref=p;respaldo=respaldo && isfinite(factor_ref) &&
        factor_ref>=0.5f && factor_ref<=2 && p>=1 && p<=20;
    persist_factor=factor;persist_ppr=ppr;persist_cal=calibrada;
    persist_ref=factor_ref;persist_refppr=ppr_ref;persist_respaldo=respaldo;
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
    ESP_ERROR_CHECK(pwm_init(500,duty,1,pwm_gpio)); /* 2 kHz */
    ESP_ERROR_CHECK(pwm_set_phase(0,0));
    ESP_ERROR_CHECK(pwm_start());
    gpio_config_t in={.pin_bit_mask=(1U<<SENSOR)|(1U<<INICIO)|(1U<<CAL),.mode=GPIO_MODE_INPUT,.pull_up_en=GPIO_PULLUP_ENABLE};
    ESP_ERROR_CHECK(gpio_config(&in));
    ESP_ERROR_CHECK(gpio_set_intr_type(SENSOR,GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(SENSOR,pulso,NULL));
    sensor_nivel=gpio_get_level(SENSOR);
    sensor_cambio=esp_timer_get_time();
    uart_config_t uart={.baud_rate=115200,.data_bits=UART_DATA_8_BITS,
        .parity=UART_PARITY_DISABLE,.stop_bits=UART_STOP_BITS_1,
        .flow_ctrl=UART_HW_FLOWCTRL_DISABLE};
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0,&uart));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0,512,0,0,NULL,0));
    lcd_init();
    printf("Tacometro ESP8266 v5 LCD1602 retencion estable filtro 2ms listo. Usar referencia real para calibrar.\n");
    printf("INICIO, PARAR, CAL, REF <rpm>, PPR <1..20>, RESTAURAR\n");
    boton_t bi={INICIO,1,1,0},bc={CAL,1,1,0};
    int64_t cal_desde=0,proxima_pantalla=0,alarma_desde=0;
    bool larga=false,alarma_previa=false;
    bool sonido_anterior=false;
    char entrada[80];unsigned len=0;
  for(;;) {
        int64_t ahora=esp_timer_get_time();
        if(boton(&bi) && bi.firme==0) {
            if(marcha) detener();
            else iniciar();
        }
        if(boton(&bc)) {
            if(bc.firme==0){cal_desde=ahora;larga=false;}
            else if(!larga) alternar_calibracion();
        }
        if(bc.firme==0 && !larga && ahora-cal_desde>=3000000) {
            larga=true;restaurar();
        }
        uint8_t ch;
        while(uart_read_bytes(UART_NUM_0,&ch,1,0)>0) {
            if(ch=='\n'||ch=='\r') {
                if(len){entrada[len]=0;comando(entrada);len=0;}
            } else if(len<sizeof entrada-1) entrada[len++]=ch;
        }
        medir();
        bool error=marcha && ahora-inicio_medicion>3000000 && (!senal||!rango||irregular);
        /* Una sola indicacion: error, calibracion o estado guardado. */
        bool alarma=marcha && (error || (!calibrada && !calibrando &&
                     ahora-inicio_medicion>=3000000));
        gpio_set_level(ROJO,!alarma);
        gpio_set_level(VERDE,!((marcha || medida_finalizada) && calibrada && !calibrando && !error));
        gpio_set_level(AZUL,marcha && calibrando && !error);
        if(alarma && !alarma_previa) {
            alarma_desde=ahora;
            pitido_fases=0;
        }
        alarma_previa=alarma;
        while(pitido_fases && ahora>=pitido_hasta) {
            pitido_fases--;
            pitido_hasta+=200000;
        }
        /* Dos pitidos: 200 ms sonido, 200 ms pausa, 200 ms sonido.
           LED rojo: 200 ms sonido / 300 ms pausa mientras siga encendido. */
        bool sonido=marcha && (alarma ? (ahora-alarma_desde)%500000<200000 :
                            pitido_fases%2!=0);
        if(sonido!=sonido_anterior) {
            ESP_ERROR_CHECK(pwm_set_duty(0,sonido?250:0));
            ESP_ERROR_CHECK(pwm_start());
            sonido_anterior=sonido;
        }
        if(ahora>=proxima_pantalla) {
            /* LCD 1602: RPM en fila 1; estado en fila 2.
               El resultado final no cambia aunque el eje se detenga. */
            char b[40];
            if(!marcha && lectura_guardada) snprintf(b,sizeof b,"RPM: %.0f",ultima_rpm);
            else if(senal && rango && !irregular) snprintf(b,sizeof b,"RPM: %.0f",rpm);
            else snprintf(b,sizeof b,"RPM: --");
            linea(0,b);
            const char *estado=medida_finalizada?"ESTABLE - INICIO":
                !marcha?"DETENIDO":calibrando?"CALIBRACION":!senal?
                "SIN SENAL":!rango?"FUERA DE RANGO":irregular?
                "SENAL IRREGULAR":estable?"LECTURA ESTABLE":"ESTABILIZANDO";
            /* Avisar si la lectura retenida aun no tiene calibracion. */
            if(medida_finalizada && !calibrada) estado="SIN CALIBRAR";
            linea(1,estado);
            proxima_pantalla=ahora+250000;
            printf("RPM=%.1f senal=%d estable=%d rango=%d pulsos=%u descartados=%u irregular=%d\n",rpm,senal,estable,rango,pulsos_ventana,descartados_ventana,irregular);
        }
        vTaskDelay(1);
    }
}
