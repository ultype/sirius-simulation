#ifndef __TRANSCEIVER_HH__
#define __TRANSCEIVER_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Master-Slave Transmission)
LIBRARY DEPENDENCY:
      ((../../src/aux/transceiver.cpp))
*******************************************************************************/
#include "rocket/Ins.hh"

#include "trick_utils/comm/include/tc.h"
#include "trick_utils/comm/include/tc_proto.h"

class Transceiver {
    public:
        Transceiver() {};

        void initialize_connection(char* name);
        void transmit(void* ptr, uint32_t size);
        uint32_t receive_size();
        void receive_data(void* ptr, uint32_t size);

    private:
        TCDevice dev;
        TrickErrorHndlr   err_hndlr;
};

#endif  // __TRANSCEIVER_HH__
