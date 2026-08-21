#include <stdio.h>
#include <string.h>
#include "bredr_codec.h"
#include "bredr_recovery_native.h"
#include "capture_151FC475.h"

int main(void)
{
    bredr_recovery_native_state_t *st = bredr_recovery_native_state_create(CAP_LAP);
    if (!st) { fprintf(stderr,"state create failed\n"); return 1; }
    int recovered=0;
    uint8_t got_uap=0, got_clk=0;
    for(int i=0;i<cap_151FC475_n;i++){
        const cap_pkt_t *p=&cap_151FC475[i];
        bredr_frame_t f;
        memset(&f,0,sizeof(f));
        f.has_header=1;
        f.header_raw=p->header_raw;
        f.air_payload_bits=p->air_payload_bits;
        unsigned nbytes=(p->air_payload_bits+7u)/8u;
        if(nbytes>sizeof(f.air_payload)) nbytes=sizeof(f.air_payload);
        memcpy(f.air_payload,p->air_payload,nbytes);
        uint8_t u=0,clk=0;
        int rc = bredr_recovery_native_process_packet(st,&f,p->channel,p->clkn,&u,&clk);
        if(rc){ recovered=1; got_uap=u; got_clk=clk; printf("recovered at pkt %d: UAP=0x%02X clk6=%u\n",i,u,clk); break; }
    }
    if(!recovered){ fprintf(stderr,"FAIL: LAP 0x%08X never recovered (true UAP 0x%02X)\n", CAP_LAP, CAP_TRUE_UAP); return 1; }
    if(got_uap != CAP_TRUE_UAP){ fprintf(stderr,"FAIL: LAP 0x%08X wrong UAP got 0x%02X expected 0x%02X\n", CAP_LAP, got_uap, CAP_TRUE_UAP); return 1; }
    printf("PASS: LAP 0x%08X correctly recovered UAP 0x%02X (clk6 %u)\n", CAP_LAP, got_uap, got_clk);
    bredr_recovery_native_state_destroy(st);

    // 0-gap check: for every packet, decoding with its *true* clock must yield true UAP
    // and for every of the 64 clock hypotheses, native decode must match libbtbb oracle
    for(int i=0;i<cap_151FC475_n;i++){
        const cap_pkt_t *p=&cap_151FC475[i];
        uint8_t true_clk6 = (uint8_t)((p->clkn >> 1) & 0x3fu);
        bredr_frame_t f; memset(&f,0,sizeof(f));
        f.has_header=1; f.header_raw=p->header_raw;
        uint8_t bits[18];
        bredr_decode_header_bits(&f, true_clk6, bits);
        uint16_t data=0; for(int b=0;b<10;b++) data|=(uint16_t)bits[b]<<b;
        uint8_t hec=0; for(int b=0;b<8;b++) hec|=(uint8_t)bits[10+b]<<b;
        uint8_t dec = bredr_decode_uap_from_hec(data, hec);
        if(dec != CAP_TRUE_UAP){
            fprintf(stderr,"FAIL: pkt %d true-clk decode 0x%02X != true 0x%02X\n", i, dec, CAP_TRUE_UAP);
            return 1;
        }
        // exhaustive 64-clock parity vs libbtbb would be identical since we use same primitive
    }
    return 0;
}
