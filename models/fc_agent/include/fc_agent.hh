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

int fc_agent_tvc_cmd_movement(struct can_frame *tvc_no1_frame, struct can_frame *tvc_no2_frame, void *data_base);

#endif  //  MODELS_FC_AGENT_INCLUDE_FC_AGENT_HH_
