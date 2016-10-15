#ifndef _lib_h
#define _lib_h


#ifdef CFG_lib


u1_t gethex (u1_t* dst, const u1_t* src, u2_t len);

u1_t puthex (u1_t* dst, const u1_t* src, u1_t len);

u1_t int2hex (u1_t* dst, u4_t v);

u4_t hex2u4 (const u1_t* src, u1_t len);

u1_t hex2int (u4_t* n, const u1_t* src, u1_t len);

u1_t dec2int (u4_t* n, const u1_t* src, u1_t len);

void reverse (u1_t* dst, const u1_t* src, u1_t len);

u1_t tolower (u1_t c);

u1_t toupper (u1_t c);

u1_t cpystr (u1_t* dst, const char* src);

u1_t cmpstr (u1_t* buf, u1_t len, char* str);

int sprintf(char *out, const char *format, ...);


#endif /* CFG_lib */

#endif /* _lib_h */
