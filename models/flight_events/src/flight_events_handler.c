#include "flight_events_handler.h"
#include "flight_events_define.h"


int flight_event_code_handler(struct icf_ctrlblk_t *C, uint64_t *event_code, int system_type) {
    struct can_frame receive_event;
    switch(system_type) {
        case ICF_SYSTEM_TYPE_SIL_EGSE:
            break;
        case ICF_SYSTEM_TYPE_EGSE:
                if (icf_rx_dequeue(C, EGSE_RX_FLIGHT_EVENT_QIDX, &receive_event, sizeof(struct can_frame)) > 0) {
                    *event_code = receive_event.data[0];
                    //  fprintf(stderr, "[sim:%f] flight_event_code_record : %d\n", receive_event.data[0], exec_get_sim_time());
                }
            break;
        default:
            fprintf(stderr, "Unknown System type %d !!!\n", system_type);
    }
    return 0;
}