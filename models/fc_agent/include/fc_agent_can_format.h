/********************************* TRICK HEADER *******************************
PURPOSE:
      Flight Computer Agent
LIBRARY DEPENDENCY:
      (
      )
PROGRAMMERS:
      (((Dung-Ru Tsai) () () () ))
*******************************************************************************/
#ifndef MODELS_FC_AGENT_INCLUDE_FC_AGENT_CAN_FORMAT_H_
#define MODELS_FC_AGENT_INCLUDE_FC_AGENT_CAN_FORMAT_H_

struct fc_agent_can_format_info_t {
    uint8_t rest_of_trigger;
    uint32_t canid;
    uint8_t taskcmd;
    uint8_t content[7];
    uint64_t event_code;
};

#endif  //  MODELS_FC_AGENT_INCLUDE_FC_AGENT_CAN_FORMAT_H_
