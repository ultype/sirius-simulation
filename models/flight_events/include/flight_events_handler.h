#ifndef MODELS_FLIGHT_EVENTS_INCLUDE_FLIGHT_EVENTS_HANDLER_H_
#define MODELS_FLIGHT_EVENTS_INCLUDE_FLIGHT_EVENTS_HANDLER_H_
/********************************* TRICK HEADER *******************************
PURPOSE:
      SDT GPSR
LIBRARY DEPENDENCY:
      (
        (../src/flight_events_handler.c)
      )
PROGRAMMERS:
      (((Dung-Ru Tsai) () () () ))
*******************************************************************************/
#include <stdint.h>
#include "icf_trx_ctrl.h"

struct flight_event_can_info_t {
    uint8_t times_of_handle;
    uint32_t canid;
    uint8_t taskcmd;
    uint8_t content[7];
    uint64_t event_code;
};

#ifdef __cplusplus
extern "C" {
#endif
int flight_event_code_handler(struct icf_ctrlblk_t *C, uint64_t *event_code, int system_type);

#ifdef __cplusplus
}
#endif

#endif  //  MODELS_FLIGHT_EVENTS_INCLUDE_FLIGHT_EVENTS_HANDLER_H_
