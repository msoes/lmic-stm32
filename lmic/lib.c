#include "oslmic.h"
#include "lib.h"


#ifdef CFG_lib


#include <stdarg.h>





static const s1_t hexval[] = {
    /*00-1F*/ -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    /*20-3F*/ -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,-1,-1,-1,-1,-1,-1,
    /*40-5F*/ -1,10,11,12,13,14,15,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    /*60-7F*/ -1,10,11,12,13,14,15,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    /*80-9F*/ -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    /*A0-BF*/ -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    /*C0-DF*/ -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    /*E0-FF*/ -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
};

u1_t gethex (u1_t* dst, const u1_t* src, u2_t len) {
    u1_t n = 0;
    if(len & 1) { // odd number of digits
	return 0;
    }
    while(len--) {
	s1_t v = hexval[*src++];
	if(v < 0) { // bad hex digit
	    return 0;
	}
	*dst = (*dst << 4) | v; // shift nibble
	if((len & 1) == 0) { // advance at every second digit
	    dst++;
	    n++;
	}
    }
    return n;
}

u1_t puthex (u1_t* dst, const u1_t* src, u1_t len) {
    u1_t l = len;
    while(len--) {
	*dst++ = "0123456789ABCDEF"[*src >> 4];
	*dst++ = "0123456789ABCDEF"[*src & 0xF];
	src++;
    }
    return 2*l;
}



u4_t hex2u4 (const u1_t* src, u1_t len) {
    u4_t n = 0;
    while(len--) {
	s1_t v = hexval[*src++];
	if(v < 0) { // bad hex digit
	    return 0;
	}
	n = (n << 4) | v; // shift nibble
    }
    return n;
}


u1_t int2hex (u1_t* dst, u4_t v) {
    u1_t tmp[4];
    tmp[0] = v >> 24;
    tmp[1] = v >> 16;
    tmp[2] = v >>  8;
    tmp[3] = v;
    puthex(dst, tmp, 4);
    return 8;
}

u1_t hex2int (u4_t* n, const u1_t* src, u1_t len) {
    *n = 0;
    while(len--) {
	s1_t v = hexval[*src++];
	if(v < 0) { // bad hex digit
	    return 0;
	}
	*n = (*n << 4) | v; // shift nibble
    }
    return 1;
}

u1_t dec2int (u4_t* n, const u1_t* src, u1_t len) {
    *n = 0;
    while(len--) {
	u1_t v = *src++;
	if(v < '0' || v > '9') { // bad decimal digit
	    return 0;
	}
	*n = (*n * 10) + v; // shift digit
    }
    return 1;
}

void reverse (u1_t* dst, const u1_t* src, u1_t len) {
    // works in-place (but not arbitrarily overlapping)
    for(u1_t i=0, j=len-1; i < j; i++, j--) {
	u1_t x = src[i];
	dst[i] = src[j];
	dst[j] = x;
    }
}

u1_t tolower (u1_t c) {
    if(c >= 'A' && c <= 'Z') {
	c += 'a' - 'A'; // make lower case
    }
    return c;
}

u1_t toupper (u1_t c) {
    if(c >= 'a' && c <= 'z') {
	c -= 'a' - 'A'; // make upper case
    }
    return c;
}

u1_t cpystr (u1_t* dst, const char* src) {
    u1_t n = 0;
    while( (*dst++ = *src++) != 0 ) n++;
    return n;
}

// compare buffer with nul-terminated string (case-insensitive)
u1_t cmpstr (u1_t* buf, u1_t len, char* str) {
    while(len--) {
	if(tolower(*buf++) != tolower(*str++)) {
	    return 0;
	}
    }
    return (*str == 0);
}




static void printchar(char **str, int c) {
    ASSERT(str);
    **str = c;
    ++(*str);
}

#define PAD_RIGHT 1
#define PAD_ZERO 2

static int prints(char **out, const char *string, int width, int pad) {
    register int pc = 0, padchar = ' ';

    if (width > 0) {
	register int len = 0;
	register const char *ptr;
	for (ptr = string; *ptr; ++ptr) ++len;
	if (len >= width) width = 0;
	else width -= len;
	if (pad & PAD_ZERO) padchar = '0';
    }
    if (!(pad & PAD_RIGHT)) {
	for ( ; width > 0; --width) {
	    printchar (out, padchar);
	    ++pc;
	}
    }
    for ( ; *string ; ++string) {
	printchar (out, *string);
	++pc;
    }
    for ( ; width > 0; --width) {
	printchar (out, padchar);
	++pc;
    }
    return pc;
}

/* the following should be enough for 32 bit int */
#define PRINT_BUF_LEN 12

static int printi(char **out, int i, int b, int sg, int width, int pad, int letbase) {
    char print_buf[PRINT_BUF_LEN];
    register char *s;
    register int t, neg = 0, pc = 0;
    register unsigned int u = i;
    
    if (i == 0) {
	print_buf[0] = '0';
	print_buf[1] = '\0';
	return prints (out, print_buf, width, pad);
    }
    
    if (sg && b == 10 && i < 0) {
	neg = 1;
	u = -i;
    }
    
    s = print_buf + PRINT_BUF_LEN-1;
    *s = '\0';
    
    while (u) {
	t = u % b;
	if( t >= 10 )
	    t += letbase - '0' - 10;
	*--s = t + '0';
	u /= b;
    }
    
    if (neg) {
	if( width && (pad & PAD_ZERO) ) {
	    printchar (out, '-');
	    ++pc;
	    --width;
	}
	else {
	    *--s = '-';
	}
    }
    
    return pc + prints (out, s, width, pad);
}



static int print(char **out, const char *format, va_list args )
{
	register int width, pad;
	register int pc = 0;
	char scr[2];

	for (; *format != 0; ++format) {
		if (*format == '%') {
			++format;
			width = pad = 0;
			if (*format == '\0') break;
			if (*format == '%') goto out;
			if (*format == '-') {
				++format;
				pad = PAD_RIGHT;
			}
			while (*format == '0') {
				++format;
				pad |= PAD_ZERO;
			}
			for ( ; *format >= '0' && *format <= '9'; ++format) {
				width *= 10;
				width += *format - '0';
			}
			if( *format == 's' ) {
				register char *s = (char *)va_arg( args, int );
				pc += prints (out, s?s:"(null)", width, pad);
				continue;
			}
			if( *format == 'd' ) {
				pc += printi (out, va_arg( args, int ), 10, 1, width, pad, 'a');
				continue;
			}
			if( *format == 'x' ) {
				pc += printi (out, va_arg( args, int ), 16, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'X' ) {
				pc += printi (out, va_arg( args, int ), 16, 0, width, pad, 'A');
				continue;
			}
			if( *format == 'u' ) {
				pc += printi (out, va_arg( args, int ), 10, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'c' ) {
				/* char are converted to int then pushed on the stack */
				scr[0] = (char)va_arg( args, int );
				scr[1] = '\0';
				pc += prints (out, scr, width, pad);
				continue;
			}
		}
		else {
		out:
			printchar (out, *format);
			++pc;
		}
	}
	if (out) **out = '\0';
	va_end( args );
	return pc;
}


int sprintf(char *out, const char *format, ...) {
    va_list args;
    
    va_start( args, format );
    return print( &out, format, args );
}




#endif /* CFG_lib */
