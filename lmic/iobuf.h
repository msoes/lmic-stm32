#ifndef _iobuf_h
#define _iobuf_h


#ifdef CFG_iobuf


// Numbr of preallocated IO buffers
#define IOBUF_CNT           4
// Capacity of each buffer.
#define IOBUF_DATA_SIZE     256

typedef struct iobuf_t iobuf_t;

// IO buffer
struct iobuf_t {
    // Next to queue buffers
    iobuf_t *next;    
    // Length of valid data in buffer
    u1_t len;
    // Buffer
    u1_t data[IOBUF_DATA_SIZE];
    // Cookie
    void *cookie;
    // Used to signal status with io buffer 
    u1_t status;
};


// Init.
void iobuf_init();
// Allocate buffer, might return NULL.
iobuf_t *iobuf_alloc();
// Release buffer.
void iobuf_free(iobuf_t *iobuf);

// Return first element in queue or NULL.
iobuf_t *iobufq_shift(iobuf_t **anchor);
// Append to queue.
void iobufq_append(iobuf_t **anchor, iobuf_t *iobuf);


# endif /* CFG_iobuf */

#endif /* _iobuf_h */

