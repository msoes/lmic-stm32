#include "oslmic.h"
#include "rb.h"


#ifdef CFG_rb


void rb_init(rb_t *rb, u1_t *buf, u2_t cnt) {
    rb->buf = buf;
    rb->cnt = cnt;
    rb->head = rb->tail = 0;
}

int rb_inuse(rb_t *rb) {
    return rb->head - rb->tail;
}

int rb_isfull(rb_t *rb) {
    return (rb_inuse(rb) == rb->cnt);
}

int rb_isempty(rb_t *rb) {
    return (rb_inuse(rb) == 0);
}

u2_t rb_avail(rb_t *rb) {
    return rb->cnt - rb_inuse(rb);
}


void rb_putc(rb_t *rb, u1_t c) {
    ASSERT(!rb_isfull(rb));
    rb->buf[rb->head++%rb->cnt] = c;
}

u1_t rb_get(rb_t *rb) {
    ASSERT(!rb_isempty(rb));
    return  rb->buf[rb->tail++%rb->cnt]; 
}

u1_t rb_peek(rb_t *rb) {
    ASSERT(!rb_isempty(rb));
    return rb->buf[rb->tail%rb->cnt]; 
}

u1_t rb_peekn(rb_t *rb, u2_t idx) {
    ASSERT(!rb_isempty(rb));
    return rb->buf[(rb->tail+idx)%rb->cnt]; 
}


/* u2_t rb_putn(rb_t *rb, u1_t *buf, u2_t cnt) { */
/*     u2_t i = 0; */
/*     for (i = 0; i < cnt; i++) { */
/* 	if (rb_isfull(rb)) { */
/* 	    return i; */
/* 	} */
/* 	rb_putc(rb, buf[i]); */
/*     } */
/*     return i; */
/* } */

u2_t rb_putn(rb_t *rb, u1_t *buf, u2_t cnt) {
    u2_t avail = rb_avail(rb);
    if (cnt > avail) {
	cnt = avail;
    }
    if (cnt <= (rb->cnt - (rb->head%rb->cnt))) {
	os_copyMem(rb->buf + (rb->head%rb->cnt), buf, cnt);
    } else {
	u2_t rightcnt = rb->cnt - (rb->head%rb->cnt);
	u2_t leftcnt = cnt - rightcnt;
	os_copyMem(rb->buf + (rb->head%rb->cnt), buf, rightcnt);
	os_copyMem(rb->buf, buf+rightcnt, leftcnt);
    }
    rb->head += cnt;
    return cnt;
}


void rb_test() {

    u1_t buf[4];
    rb_t rb;

    rb_t *rbp = &rb;
    rb_init(rbp, buf, 4);
    ASSERT(rb_isempty(rbp));
    rb_putc(rbp, 0x1);
    ASSERT(rb_inuse(rbp) == 1);
    ASSERT(!rb_isempty(rbp));
    ASSERT(!rb_isfull(rbp));
    rb_putc(rbp, 0x2);
    ASSERT(rb_inuse(rbp) == 2);
    ASSERT(!rb_isempty(rbp));
    ASSERT(!rb_isfull(rbp));
    rb_putc(rbp, 0x3);
    ASSERT(rb_inuse(rbp) == 3);
    ASSERT(!rb_isempty(rbp));
    ASSERT(!rb_isfull(rbp));
    rb_putc(rbp, 0x4);
    ASSERT(rb_inuse(rbp) == 4);
    ASSERT(rb_isfull(rbp));

    ASSERT(rb_peek(rbp) == 1);
    u1_t u = rb_get(rbp);
    ASSERT(u == 1);
    ASSERT(rb_inuse(rbp) == 3);
    ASSERT(!rb_isempty(rbp));
    ASSERT(!rb_isfull(rbp));

    ASSERT(rb_peek(rbp) == 2);    
    u = rb_get(rbp);
    ASSERT(u == 2);
    ASSERT(rb_inuse(rbp) == 2);
    ASSERT(!rb_isempty(rbp));
    ASSERT(!rb_isfull(rbp));

    ASSERT(rb_peek(rbp) == 3);    
    u = rb_get(rbp);
    ASSERT(u == 3);
    ASSERT(rb_inuse(rbp) == 1);
    ASSERT(!rb_isempty(rbp));
    ASSERT(!rb_isfull(rbp));

    ASSERT(rb_peek(rbp) == 4);    
    u = rb_get(rbp);
    ASSERT(u == 4);
    ASSERT(rb_inuse(rbp) == 0);
    ASSERT(rb_isempty(rbp));
    ASSERT(!rb_isfull(rbp));

    rb_putc(rbp, 0xFF);
    ASSERT(rb_inuse(rbp) == 1);
    u = rb_get(rbp);
    ASSERT(u == 0xFF);
    
    rb_putc(rbp, 0x1);
    ASSERT(rb_inuse(rbp) == 1);
    ASSERT(rbp->head==6);

    u1_t buf2[4];
    buf2[0] = 0x5;
    buf2[1] = 0x6;
    buf2[2] = 0x7;
    buf2[3] = 0x8;

    u = (u1_t) rb_putn(rbp, buf2, 2);
    ASSERT(rb_inuse(rbp) == 3);
    ASSERT(rb_peekn(rbp, 0) == 0x1);
    ASSERT(rb_peekn(rbp, 1) == 0x5);
    ASSERT(rb_peekn(rbp, 2) == 0x6);

    u = rb_get(rbp);
    ASSERT(u == 1);
    ASSERT(rb_inuse(rbp) == 2);
    ASSERT(rb_peekn(rbp, 0) == 0x5);
    ASSERT(rb_peekn(rbp, 1) == 0x6);
    
    buf2[0] = 0x11;
    buf2[1] = 0x13;
    u = (u1_t) rb_putn(rbp, buf2, 2);
    ASSERT(rb_inuse(rbp) == 4);
    ASSERT(rb_peekn(rbp, 0) == 0x5);
    ASSERT(rb_peekn(rbp, 1) == 0x6);
    ASSERT(rb_peekn(rbp, 2) == 0x11);
    ASSERT(rb_peekn(rbp, 3) == 0x13);
}


#endif /* CFG_rb */
