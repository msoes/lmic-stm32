#include "oslmic.h"
#include "lfsr113.h"


#ifdef CFG_lfsr113


static u4_t lfsr113_z1 = 12345, lfsr113_z2 = 12345, lfsr113_z3 = 12345, lfsr113_z4 = 12345;


u4_t lfsr113_Bits (void) {
    u4_t b;
    b  = ((lfsr113_z1 << 6) ^ lfsr113_z1) >> 13;
    lfsr113_z1 = ((lfsr113_z1 & 4294967294U) << 18) ^ b;
    b  = ((lfsr113_z2 << 2) ^ lfsr113_z2) >> 27; 
    lfsr113_z2 = ((lfsr113_z2 & 4294967288U) << 2) ^ b;
    b  = ((lfsr113_z3 << 13) ^ lfsr113_z3) >> 21;
    lfsr113_z3 = ((lfsr113_z3 & 4294967280U) << 7) ^ b;
    b  = ((lfsr113_z4 << 3) ^ lfsr113_z4) >> 12;
    lfsr113_z4 = ((lfsr113_z4 & 4294967168U) << 13) ^ b;
    return (lfsr113_z1 ^ lfsr113_z2 ^ lfsr113_z3 ^ lfsr113_z4);
}


void  lfsr113_seed (u2_t seed) {
    lfsr113_z1 = lfsr113_z2 = lfsr113_z3 = lfsr113_z4 = seed;
}


#endif /* CFG_lfsr113 */
