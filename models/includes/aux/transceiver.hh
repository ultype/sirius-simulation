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
        void transmit(INS& ins);
        void receive(INS& ins);

    private:
        TCDevice dev;
        TrickErrorHndlr   err_hndlr;
};

#endif  // __TRANSCEIVER_HH__
