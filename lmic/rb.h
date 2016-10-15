#ifndef _rb_h
#define _rb_h


#ifdef CFG_rb


typedef struct {
    u2_t head;
    u2_t tail;
    u1_t *buf;
    u2_t cnt;
} rb_t;


void rb_init(rb_t *rb, u1_t *buf, u2_t cnt);

int rb_inuse(rb_t *rb);

u2_t rb_avail(rb_t *rb);

int rb_isfull(rb_t *rb);

int rb_isempty(rb_t *rb);

void rb_putc(rb_t *rb, u1_t c);

u2_t rb_putn(rb_t *rb, u1_t *buf, u2_t cnt);

u1_t rb_get(rb_t *rb);

u1_t rb_peek(rb_t *rb);

u1_t rb_peekn(rb_t *rb, u2_t idx);

void rb_test();



#endif /* CFG_rb */

#endif /* _rb_h */
