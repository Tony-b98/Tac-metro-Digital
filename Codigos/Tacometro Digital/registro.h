
#pragma once
/* Registro de mediciones en la flash (particion SPIFFS "storage").
   Cada medicion finalizada se guarda como un registro de tamano fijo;
   la planilla (CSV) y la lista para la app (JSON) se generan al pedirlas.
   Las funciones que escriben deben llamarse solo desde el lazo principal. */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

/* ===== Constantes ===== */
#define REGISTRO_MAXIMO 1000
#define REGISTRO_OBS_MAX 48   /* bytes de observacion del usuario, con el 0 final */

/* ===== Tipos ===== */
typedef struct {
    uint32_t numero;          /* Nº de medicion, desde 1 */
    uint32_t epoch;           /* fecha y hora UTC (segundos); 0 = sin hora */
    uint32_t uptime_s;        /* segundos desde el arranque al medir */
    uint16_t arranque;        /* numero de arranque del equipo al medir */
    int16_t zona_min;         /* minutos respecto de UTC (Uruguay: -180) */
    uint16_t ppr;
    uint8_t calibrada;
    uint8_t reservado;
    float rpm;
    float factor;
    char obs[REGISTRO_OBS_MAX];
} registro_t;

/* Destino de la planilla o la lista: HTTP, puerto serie, etc. */
typedef esp_err_t (*registro_salida_t)(void *ctx,const char *texto,size_t n);

/* ===== Prototipos ===== */
esp_err_t registro_iniciar(void);
unsigned registro_cantidad(void);
esp_err_t registro_agregar(registro_t *r);
esp_err_t registro_observacion(uint32_t numero,const char *texto);
esp_err_t registro_completar_hora(uint16_t arranque,uint32_t epoch_ahora,
                                  uint32_t uptime_ahora,int16_t zona_min);
esp_err_t registro_borrar(void);
esp_err_t registro_csv(registro_salida_t salida,void *ctx,bool bom);
esp_err_t registro_json(registro_salida_t salida,void *ctx);
void registro_limpiar_texto(char *destino,const char *origen,size_t n);


