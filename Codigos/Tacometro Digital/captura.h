#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

#define CAPTURA_TIMEOUT_US 3000000LL
#define CAPTURA_VENTANA_US 1000000LL

typedef struct {
    int64_t ultimo;
    uint64_t suma_us;
    uint32_t periodos, descartados, minimo_us, maximo_us;
} captura_t;

static inline void captura_limpiar(volatile captura_t *c) {
    c->suma_us=0; c->periodos=0; c->descartados=0;
    c->minimo_us=UINT32_MAX; c->maximo_us=0;
}
static inline void captura_reset(volatile captura_t *c) {
    c->ultimo=0; captura_limpiar(c);
}
static inline void captura_evento(volatile captura_t *c,int64_t ahora,unsigned ppr) {
    if(!c->ultimo) {c->ultimo=ahora; return;}
    int64_t dt=ahora-c->ultimo;
    if(dt<=0) {c->descartados++; return;}
    /* 24000 RPM crudas: admite factor de calibracion hasta 0.5.
       No mover ultimo al rechazar un rebote corto. */
    unsigned minimo=2500U/ppr;
    if(minimo<200) minimo=200;
    if(dt<(int64_t)minimo) {c->descartados++; return;}
    c->ultimo=ahora;
    if(dt>=CAPTURA_TIMEOUT_US) {captura_limpiar(c); return;}
    c->periodos++; c->suma_us+=(uint32_t)dt;
    if((uint32_t)dt<c->minimo_us) c->minimo_us=(uint32_t)dt;
    if((uint32_t)dt>c->maximo_us) c->maximo_us=(uint32_t)dt;
}
typedef struct {float cruda,rpm; bool rango,regular;} resultado_t;
static inline resultado_t captura_resultado(const captura_t *c,unsigned ppr,float factor) {
    resultado_t r={0};
    if(!c->periodos || !c->suma_us || !ppr) return r;
    r.cruda=(float)(60000000.0*(double)c->periodos/((double)c->suma_us*ppr));
    r.rpm=r.cruda*factor;
    r.rango=r.rpm>=60.0f && r.rpm<=12000.0f;
    double promedio=(double)c->suma_us/c->periodos;
    /* No declarar estable una entrada con muchos pulsos descartados. */
    r.regular=(c->maximo_us-c->minimo_us)<=promedio*0.10 &&
              (uint64_t)c->descartados*4<=c->periodos;
    return r;
}
