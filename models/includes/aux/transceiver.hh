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

#include <functional>
#include <map>
#include <set>
#include <list>

class Transceiver {
    public:
        Transceiver() {};

        void initialize_connection(char* name);

        void begin_transmit();
        void register_for_transmit_double(std::function<double()> in);
        void register_for_transmit_vec3(std::function<arma::vec3()> in);
        void register_for_transmit_mat33(std::function<arma::mat33()> in);
        void transmit();

        uint32_t begin_receive();
        std::function<double()> register_to_receive_double(std::string name);
        std::function<arma::vec3()> register_to_receive_vec3(std::string name);
        std::function<arma::mat33()> register_to_receive_mat33(std::string name);

        double get_double(std::string id);
        arma::vec3 get_vec3(std::string id);
        arma::mat33 get_mat33(std::string id);

    private:
        TCDevice dev;
        TrickErrorHndlr   err_hndlr;

        std::list<double> data_out;

        std::set<std::string> data_in_tag;
        std::map<std::string, double> data_in_double;
        std::map<std::string, arma::vec3> data_in_vec3;
        std::map<std::string, arma::mat33> data_in_mat33;
};

#endif  // __TRANSCEIVER_HH__
