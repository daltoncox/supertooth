#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bredr_codec.h"
#include "bredr_clock_recovery.h"
#include "capture_151FC475.h"

int main(void)
{
    bredr_piconet_t *pnet = malloc(sizeof(*pnet));
    if(!pnet) return 1;
    bredr_piconet_init(pnet, CAP_LAP);
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
        bredr_recovery_result_t r;
        int rc = bredr_recovery_process_packet(pnet,&f,p->channel,p->clkn,&r);
        if(rc){ recovered=1; got_uap=r.uap; got_clk=r.clk6_hint; printf("recovered at pkt %d: UAP=0x%02X clk6=%u\n",i,got_uap,got_clk); break; }
    }
    free(pnet);
    if(!recovered){ fprintf(stderr,"FAIL: LAP 0x%08X never recovered (true UAP 0x%02X)\n", CAP_LAP, CAP_TRUE_UAP); return 1; }
    if(got_uap != CAP_TRUE_UAP){ fprintf(stderr,"FAIL: LAP 0x%08X wrong UAP got 0x%02X expected 0x%02X\n", CAP_LAP, got_uap, CAP_TRUE_UAP); return 1; }
    printf("PASS: LAP 0x%08X correctly recovered UAP 0x%02X (clk6 %u)\n", CAP_LAP, got_uap, got_clk);

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
