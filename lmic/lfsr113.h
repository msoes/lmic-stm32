#ifndef _lfsr113_h
#define _lfsr113_h

#ifdef CFG_lfsr113

/* 
 * 32-bits Random number generator U[0,1): lfsr113
 * Author: Pierre L'Ecuyer,
 * Source: http://www.iro.umontreal.ca/~lecuyer/myftp/papers/tausme2.ps 
*/
void lfsr113_seed (u2_t seed);

u4_t lfsr113_rand (void);


#endif /* CFG_lfsr113 */

#endif /* _lfsr113_h */
