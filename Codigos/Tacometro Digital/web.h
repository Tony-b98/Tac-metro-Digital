
#pragma once
/* WiFi, hora y app web del tacometro.
   El equipo se conecta como cliente al hotspot del celular. Desde el
   navegador del celular se abre http://<IP del LCD> y se ve la medicion
   en vivo, la lista de mediciones y la descarga de la planilla Excel.
   El servidor corre en su propia tarea: no escribe la flash, solo deja
   pedidos que el lazo principal aplica con la medicion detenida. */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "registro.h"

/* ===== Tipos ===== */
/* Foto del estado que publica el lazo principal para la app. */
typedef struct {
    bool rpm_valida;
    float rpm;
    char estado[17];
    bool marcha, calibrando, calibrada, finalizada;
    unsigned ppr;
    float factor;
    unsigned registros;
} web_estado_t;

/* Cambios pedidos desde la app, pendientes de aplicar. */
typedef struct {
    bool observacion;
    uint32_t numero;
    char texto[REGISTRO_OBS_MAX];
    bool borrar, cal, referencia;
    float ref;
} web_pedido_t;

/* ===== Prototipos ===== */
void web_iniciar(const char *ssid,const char *clave);
void web_cambiar_red(const char *ssid,const char *clave);
void web_publicar(const web_estado_t *e);
void web_responder(const char *error);
bool web_tomar_pedido(web_pedido_t *p);
bool web_ip(char *b,size_t n);
bool web_hora_valida(void);
int16_t web_zona_min(void);


