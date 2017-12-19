/********************************* TRICK HEADER *******************************
PURPOSE:
      RX Ctrl
LIBRARY DEPENDENCY:
      (
        (../src/ringbuffer.c)
      )
PROGRAMMERS:
      (((Dung-Ru Tsai) () () () ))
*******************************************************************************/
#ifndef MODELS_ICF_INCLUDE_RINGBUFFER_H_
#define MODELS_ICF_INCLUDE_RINGBUFFER_H_
#include "icf_utility.h"
#define NUM_OF_CELL  256  //  power of 2
#define GET_RINGCELL_IDX(idx)      ((idx) & (NUM_OF_CELL - 1))

struct ringbuffer_t {
        volatile uint32_t writer_idx;
        volatile uint32_t reader_idx;
        volatile uint32_t elem_num;
        uint32_t ring_size;
        uint32_t full_cnt;
        pthread_mutex_t ring_lock;
        void *pCell[NUM_OF_CELL];
};

struct ringbuffer_cell_t {
    uint32_t frame_full_size;
    void *l2frame; 
};

#ifdef __cplusplus
extern "C" {
#endif
int32_t rb_init(struct ringbuffer_t *rb, uint32_t size);
void rb_deinit(struct ringbuffer_t *rb);
void rb_push(struct ringbuffer_t *rb, void *payload);
void *rb_pop(struct ringbuffer_t *rb);
#ifdef __cplusplus
}
#endif
#endif   //  MODELS_ICF_INCLUDE_RINGBUFFER_H_
