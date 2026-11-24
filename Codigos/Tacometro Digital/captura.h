
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include "esp_attr.h"

#define CAPTURA_TIMEOUT_US 3000000LL
#define CAPTURA_VENTANA_US 1000000LL
#define CAPTURA_TOLERANCIA 0.10
#define CAPTURA_MINIMO_US 2500U
#define CAPTURA_MINIMO_PISO_US 100U

typedef struct {
    int64_t ultimo;
    uint64_t suma_us;
    uint32_t periodos, descartados;
    uint32_t minimo_us, maximo_us;
    uint32_t periodo_us;
    uint32_t vuelta_us, vuelta_marcas;
    uint64_t vueltas_suma_us;
    uint32_t vueltas;
} captura_t;

typedef struct {float cruda,rpm; bool rango,regular;} resultado_t;

static inline IRAM_ATTR void captura_limpiar(volatile captura_t *c) {
    c->suma_us=0; c->periodos=0; c->descartados=0;
    c->minimo_us=UINT32_MAX; c->maximo_us=0;
    c->vueltas_suma_us=0; c->vueltas=0;
}

static inline void captura_reset(volatile captura_t *c) {
    c->ultimo=0; c->periodo_us=0;
    c->vuelta_us=0; c->vuelta_marcas=0;
    captura_limpiar(c);
}

static inline IRAM_ATTR void captura_evento(volatile captura_t *c,int64_t ahora,unsigned ppr) {
    if(!ppr) return;
    if(!c->ultimo) {c->ultimo=ahora; return;}
    int64_t dt=ahora-c->ultimo;
    if(dt<=0) {c->descartados++; return;}
    unsigned minimo=CAPTURA_MINIMO_US/ppr;
    if(minimo<CAPTURA_MINIMO_PISO_US) minimo=CAPTURA_MINIMO_PISO_US;
    if(dt<(int64_t)minimo) {c->descartados++; return;}
    c->ultimo=ahora;
    if(dt>=CAPTURA_TIMEOUT_US) {
        c->periodo_us=0; c->vuelta_us=0; c->vuelta_marcas=0;
        captura_limpiar(c);
        return;
    }
    uint32_t p=(uint32_t)dt;
    c->periodo_us=p;
    c->periodos++; c->suma_us+=p;
    c->vuelta_us+=p; c->vuelta_marcas++;
    if(c->vuelta_marcas>=ppr) {
        uint32_t v=c->vuelta_us;
        c->vueltas++; c->vueltas_suma_us+=v;
        if(v<c->minimo_us) c->minimo_us=v;
        if(v>c->maximo_us) c->maximo_us=v;
        c->vuelta_us=0; c->vuelta_marcas=0;
    }
}

static inline resultado_t captura_resultado(const captura_t *c,unsigned ppr,float factor) {
    resultado_t r={0};
    if(!c->periodos || !c->suma_us || !ppr) return r;
    if(c->vueltas && c->vueltas_suma_us)
        r.cruda=(float)(60000000.0*(double)c->vueltas/(double)c->vueltas_suma_us);
    else
        r.cruda=(float)(60000000.0*(double)c->periodos/((double)c->suma_us*ppr));
    r.rpm=r.cruda*factor;
    r.rango=r.rpm>=60.0f && r.rpm<=12000.0f;
    bool pocos_descartes=(uint64_t)c->descartados*4<=c->periodos;
    if(c->vueltas) {
        double promedio=(double)c->vueltas_suma_us/c->vueltas;
        r.regular=pocos_descartes &&
                  (double)(c->maximo_us-c->minimo_us)<=promedio*CAPTURA_TOLERANCIA;
    } else {
        r.regular=pocos_descartes;
    }
    return r;
}


