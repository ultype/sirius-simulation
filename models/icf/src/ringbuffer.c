#include "ringbuffer.h"


int32_t rb_init(struct ringbuffer_t *rb, uint32_t size)
{
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
    pthread_cond_init(&rb->cond_space, NULL);
    pthread_cond_init(&rb->cond_count, NULL);
    return 0;
}

void rb_deinit(struct ringbuffer_t *rb)
{
    pthread_mutex_destroy(&rb->ring_lock);
    pthread_cond_destroy(&rb->cond_count);
    pthread_cond_destroy(&rb->cond_space);
    fprintf(stderr, "[%s] Full count: %d \n", __FUNCTION__, rb->full_cnt);
}

void rb_put(struct ringbuffer_t *rb, void *payload)
{
        pthread_mutex_lock(&rb->ring_lock);
        while (rb->writer_idx == (rb->ring_size + rb->reader_idx)) // ring buffer is full
        { 
            pthread_cond_wait(&rb->cond_space, &rb->ring_lock);
            rb->full_cnt++;
        }
        rb->pCell[GET_RINGCELL_IDX(rb->writer_idx)] = payload;
        rb->writer_idx++;
        rb->elem_num++;
        fprintf(stderr, "[%s].\n", __FUNCTION__);
        pthread_mutex_unlock(&rb->ring_lock);
        pthread_cond_signal(&rb->cond_count);

}

void *rb_get(struct ringbuffer_t *rb)
{
        void *ret;
        pthread_mutex_lock(&rb->ring_lock);
        while (rb->writer_idx == rb->reader_idx) //ring buffer is empty
        {
            fprintf(stderr, "[dungru:%d:%s] \n", __LINE__, __FUNCTION__);
            //pthread_mutex_unlock(&rb->ring_lock);
            pthread_cond_wait(&rb->cond_count,  &rb->ring_lock);
            fprintf(stderr, "[dungru:%d:%s] \n", __LINE__, __FUNCTION__);
        }
        ret = rb->pCell[GET_RINGCELL_IDX(rb->reader_idx)];
        rb->reader_idx++;
        rb->elem_num--;
        pthread_mutex_unlock(&rb->ring_lock);
        pthread_cond_signal(&rb->cond_space);
        return ret;
}