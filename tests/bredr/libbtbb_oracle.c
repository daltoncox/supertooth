/* libbtbb_oracle.c -- test-only oracle, copies from libbtbb bluetooth_packet.c */
#include "libbtbb_oracle.h"
#include <stdlib.h>
static const uint8_t WHITENING_DATA[] = {1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1};
static const uint8_t INDICES[] = {99, 85, 17, 50, 102, 58, 108, 45, 92, 62, 32, 118, 88, 11, 80, 2, 37, 69, 55, 8, 20, 40, 74, 114, 15, 106, 30, 78, 53, 72, 28, 26, 68, 7, 39, 113, 105, 77, 71, 25, 84, 49, 57, 44, 61, 117, 10, 1, 123, 124, 22, 125, 111, 23, 42, 126, 6, 112, 76, 24, 48, 43, 116, 0};
static const uint16_t fec23_gen_matrix[] = {0x2c01, 0x5802, 0x1c04, 0x3808, 0x7010, 0x4c20, 0x3440, 0x6880, 0x7d00, 0x5600};

uint8_t oracle_reverse(uint8_t byte){ return (byte & 0x80) >>7 | (byte &0x40)>>5 | (byte&0x20)>>3 | (byte&0x10)>>1 | (byte&0x08)<<1 | (byte&0x04)<<3 | (byte&0x02)<<5 | (byte&0x01)<<7; }
uint8_t oracle_air_to_host8(char *air,int bits){ int i; uint8_t h=0; for(i=0;i<bits;i++) h|=((uint8_t)air[i]<<i); return h; }
uint16_t oracle_air_to_host16(char *air,int bits){ int i; uint16_t h=0; for(i=0;i<bits;i++) h|=((uint16_t)air[i]<<i); return h; }

int oracle_unfec13(char *input,char *output,int length){
    int a,b,c,i,be=0;
    for(i=0;i<length;i++){ a=3*i; b=a+1; c=a+2; output[i]=((input[a]&input[b])|(input[b]&input[c])|(input[c]&input[a])); be+=((input[a]^input[b])|(input[b]^input[c])|(input[c]^input[a])); }
    return (be < (length/4));
}
static uint16_t fec23(uint16_t data){ int i; uint16_t cw=0; for(i=0;i<10;i++) if(data & (1<<i)) cw ^= fec23_gen_matrix[i]; return cw; }
char* oracle_unfec23(char *input,int length){
    int iptr,optr,count;
    char *output;
    uint8_t diff,check;
    uint16_t data,codeword;
    diff=length%10; if(0!=diff) length+=(10-diff);
    output=(char*)malloc(length);
    for(iptr=0,optr=0;optr<length;iptr+=15,optr+=10){
        for(count=0;count<10;count++) output[optr+count]=input[iptr+count];
        data=oracle_air_to_host16(input+iptr,10);
        check=oracle_air_to_host8(input+iptr+10,5);
        codeword=fec23(data);
        diff=check ^ (codeword>>10);
        if(diff & (diff-1)){
            switch(diff){
                case 0x0b: output[optr]^=1; break;
                case 0x16: output[optr+1]^=1; break;
                case 0x07: output[optr+2]^=1; break;
                case 0x0e: output[optr+3]^=1; break;
                case 0x1c: output[optr+4]^=1; break;
                case 0x13: output[optr+5]^=1; break;
                case 0x0d: output[optr+6]^=1; break;
                case 0x1a: output[optr+7]^=1; break;
                case 0x1f: output[optr+8]^=1; break;
                case 0x15: output[optr+9]^=1; break;
                default: free(output); return 0;
            }
        }
    }
    return output;
}
void oracle_unwhiten(char *input,char *output,int clock,int length,int skip){
    int count,index;
    index=INDICES[clock&0x3f]; index+=skip; index%=127;
    for(count=0;count<length;count++){ output[count]=input[count]^WHITENING_DATA[index]; index=(index+1)%127; }
}
uint8_t oracle_uap_from_hec(uint16_t data,uint8_t hec){
    int i;
    for(i=9;i>=0;i--){ if(hec&0x80) hec^=0x65; hec=(hec<<1)|(((hec>>7)^(data>>i))&0x01); }
    return oracle_reverse(hec);
}
uint16_t oracle_crcgen(char *payload,int length,int UAP){
    char bit; uint16_t reg,count;
    reg=(oracle_reverse((uint8_t)UAP)<<8)&0xff00;
    for(count=0;count<(uint16_t)length;count++){ bit=payload[count]; reg=(reg>>1)|(((reg&0x0001)^(bit&0x01))<<15); reg^=((reg&0x8000)>>5); reg^=((reg&0x8000)>>12); }
    return reg;
}
char* oracle_fec23_encode(const char *data_bits,int length){
    int blocks=length/10;
    char *out=(char*)calloc((size_t)blocks*15,1);
    if(!out) return 0;
    for(int b=0;b<blocks;b++){
        uint16_t data=oracle_air_to_host16((char*)data_bits+b*10,10);
        uint16_t cw=fec23(data);
        for(int i=0;i<15;i++) out[b*15+i]=(char)((cw>>i)&1u);
    }
    return out;
}
