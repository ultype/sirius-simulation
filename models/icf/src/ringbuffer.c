#include "ringbuffer.h"


int32_t rb_init(struct ringbuffer_t *rb, uint32_t size) {
    int idx;

    rb->writer_idx = 0;
    rb->reader_idx = 0;
    rb->elem_num = 0;
    rb->ring_size = size;
    rb->full_cnt = 0;
    for (idx = 0; idx < rb->ring_size; idx++) {
        rb->pCell[idx] = NULL;
    }
    pthread_mutex_init(&rb->ring_lock, NULL);
    return 0;
}

void rb_deinit(struct ringbuffer_t *rb) {
    pthread_mutex_destroy(&rb->ring_lock);
    fprintf(stderr, "[%s] Full count: %d \n", __FUNCTION__, rb->full_cnt);
}

void rb_push(struct ringbuffer_t *rb, void *payload) {
        pthread_mutex_lock(&rb->ring_lock);
        if (rb->writer_idx == (rb->ring_size + rb->reader_idx)) {
            //  ring buffer is full
            pthread_mutex_unlock(&rb->ring_lock);
            rb->full_cnt++;
            return;
        }
        rb->pCell[GET_RINGCELL_IDX(rb->writer_idx)] = payload;
        rb->writer_idx++;
        rb->elem_num++;
        pthread_mutex_unlock(&rb->ring_lock);
}

void *rb_pop(struct ringbuffer_t *rb) {
        void *ret;
        pthread_mutex_lock(&rb->ring_lock);
        if (rb->writer_idx == rb->reader_idx) {  //  ring buffer is empty
            pthread_mutex_unlock(&rb->ring_lock);
            return NULL;
        }
        ret = rb->pCell[GET_RINGCELL_IDX(rb->reader_idx)];
        rb->reader_idx++;
        rb->elem_num--;
        pthread_mutex_unlock(&rb->ring_lock);
        return ret;
}
