/* ADCS binary wire v1. Identical copies live in all three projects.
 * COBS(header|payload|CRC16-CCITT-FALSE), followed by zero. See PROTOCOL.md. */
#ifndef ADCS_WIRE_H
#define ADCS_WIRE_H
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#define WIRE_MAX 64U
#define WIRE_VERSION 1U
enum { WIRE_M, WIRE_C, WIRE_G, WIRE_S, WIRE_B, WIRE_T, WIRE_A };
typedef struct { uint8_t type, payload[48]; size_t length; } WireFrame;
typedef struct { uint8_t data[WIRE_MAX]; size_t used; bool discard; } WireDecoder;
static inline size_t Wire_Size(unsigned type) {
    const uint8_t sizes[]={14,17,13,11,10,10,29};
    return type<7 ? sizes[type] : 0;
}
static inline uint32_t Wire_Get(const uint8_t *p,unsigned n) {
    uint32_t v=0; for(unsigned i=0;i<n;++i) v|=(uint32_t)p[i]<<(8*i); return v;
}
static inline int32_t Wire_Signed(const uint8_t *p,unsigned n) {
    uint32_t v=Wire_Get(p,n), sign=1U<<(n*8-1);
    return (v&sign) ? (int32_t)((int64_t)v-((int64_t)1<<(n*8))) : (int32_t)v;
}
static inline void Wire_Put(uint8_t *p,uint32_t v,unsigned n) {
    for(unsigned i=0;i<n;++i) p[i]=(uint8_t)(v>>(8*i));
}
static inline uint16_t Wire_Crc(const uint8_t *p,size_t n) {
    uint16_t c=0xffff;
    while(n--) { c^=(uint16_t)*p++<<8; for(unsigned i=0;i<8;++i) c=(c&0x8000)?(uint16_t)((c<<1)^0x1021):(uint16_t)(c<<1); }
    return c;
}
static inline size_t Wire_Encode(uint8_t *out,size_t capacity,unsigned type,const uint8_t *payload) {
    size_t n=Wire_Size(type); if(!n || capacity<n+5) return 0;
    uint8_t raw[WIRE_MAX]; raw[0]=(WIRE_VERSION<<3)|type; memcpy(raw+1,payload,n);
    Wire_Put(raw+n+1,Wire_Crc(raw,n+1),2); n+=3;
    size_t code_at=0, used=1; uint8_t code=1;
    for(size_t i=0;i<n;++i) {
        if(raw[i]) { out[used++]=raw[i]; ++code; }
        else { out[code_at]=code; code_at=used++; code=1; }
    }
    out[code_at]=code; out[used++]=0; return used;
}
/* 1 complete valid frame, 0 incomplete/empty delimiter, -1 discarded frame.
 * Overflow remains discarded until a delimiter; corrupt suffixes cannot execute. */
static inline int Wire_Feed(WireDecoder *d,uint8_t byte,WireFrame *f) {
    if(byte) {
        if(!d->discard) { if(d->used==WIRE_MAX) d->discard=true; else d->data[d->used++]=byte; }
        return 0;
    }
    size_t n=d->used; bool bad=d->discard; d->used=0; d->discard=false;
    if(bad) return -1;
    if(!n) return 0;
    uint8_t raw[WIRE_MAX]; size_t i=0,k=0;
    while(i<n) {
        unsigned code=d->data[i++];
        if(!code || i+code-1>n) return -1;
        for(unsigned j=1;j<code;++j) { if(k==WIRE_MAX) return -1; raw[k++]=d->data[i++]; }
        if(code!=255 && i<n) { if(k==WIRE_MAX) return -1; raw[k++]=0; }
    }
    if(k<3 || raw[0]>>3!=WIRE_VERSION) return -1;
    unsigned type=raw[0]&7; size_t length=Wire_Size(type);
    if(!length || k!=length+3 || Wire_Get(raw+k-2,2)!=Wire_Crc(raw,k-2)) return -1;
    f->type=type; f->length=length; memcpy(f->payload,raw+1,length); return 1;
}
#endif
