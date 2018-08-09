/********************************* TRICK HEADER *******************************
PURPOSE:
      Flight Computer Agent
LIBRARY DEPENDENCY:
      (
        (../src/fc_agent.cpp)
      )
PROGRAMMERS:
      (((Dung-Ru Tsai) () () () ))
*******************************************************************************/
#ifndef MODELS_FC_AGENT_INCLUDE_FC_AGENT_HH_
#define MODELS_FC_AGENT_INCLUDE_FC_AGENT_HH_
#include  "icf_trx_ctrl.h"
#include  "DM_FSW_Interface.hh"
#include  "fc_agent_can_format.h"
#include "flight_events_define.h"

int fc_agent_tvc_cmd_movement(struct can_frame *tvc_no1_frame, struct can_frame *tvc_no2_frame, void *data_base);
int fc_agent_flight_event_can_cmd_generate(struct can_frame *event_frame, uint64_t event_code);

#endif  //  MODELS_FC_AGENT_INCLUDE_FC_AGENT_HH_
