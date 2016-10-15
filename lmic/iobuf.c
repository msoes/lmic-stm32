#include "oslmic.h"
#include "iobuf.h"


#ifdef CFG_iobuf


static iobuf_t iobuf_buffers[IOBUF_CNT];

static iobuf_t *iobuf_freeq;

void iobuf_init() {
    int i;
    for (i = 0; i < IOBUF_CNT-1; i++) {
	iobuf_buffers[i].next = &iobuf_buffers[i+1];
    }
    iobuf_buffers[IOBUF_CNT-1].next = NULL;

    iobuf_freeq = &iobuf_buffers[0];
}

iobuf_t *iobuf_alloc() {
    iobuf_t *iobuf;
    hal_disableIRQs();
    iobuf = iobuf_freeq;
    if (iobuf != NULL) {
	iobuf_freeq = iobuf->next;
	iobuf->next = NULL;
	iobuf->len = 0;
	iobuf->cookie = NULL;
    }
    hal_enableIRQs();
    return iobuf;
}

void iobuf_free(iobuf_t *iobuf) {
    hal_disableIRQs();
    iobuf->len = 0;
    iobuf->next = iobuf_freeq;
    iobuf_freeq = iobuf;
    hal_enableIRQs();
}


iobuf_t *iobufq_shift(iobuf_t **anchor) {
    iobuf_t *iobuf = *anchor;
    if (iobuf == NULL) {
	return NULL;
    }
    *anchor = iobuf->next;
    iobuf->next = NULL;
    return iobuf;
}


void iobufq_append(iobuf_t **anchor, iobuf_t *iobuf) {
    while(*anchor != NULL) {
	anchor = &(*anchor)->next;
    }
    iobuf->next = NULL;
    *anchor = iobuf;
}



#endif /* CFG_iobuf */
