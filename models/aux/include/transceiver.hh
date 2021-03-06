#ifndef __TRANSCEIVER_HH__
#define __TRANSCEIVER_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Master-Slave Transmission)
LIBRARY DEPENDENCY:
      ((../src/transceiver.cpp))
*******************************************************************************/
#include "trick_utils/comm/include/tc.h"
#include "trick_utils/comm/include/tc_proto.h"

#include <armadillo>

#include <functional>
#include <map>
#include <set>
#include <list>
#include "../../dm/include/GPS_constellation.hh"
#include "../../gnc/include/DM_FSW_Interface.hh"

class Transceiver;
class TransceiverProxy;

class Transceiver {
 public:
    Transceiver() {}

    void initialize_connection(char* name);

    void register_for_transmit(std::string cid, std::string id, std::function<double()> in);
    void register_for_transmit(std::string cid, std::string id, std::function<arma::mat()> in);
    void register_for_transmit(std::string cid, std::string id, std::function<transmit_channel*()> in);
    void register_for_transmit(std::string cid, std::string id, std::function<refactor_downlink_packet_t()> in);
    void register_for_transmit(std::string cid, std::string id, std::function<refactor_uplink_packet_t()> in);

    void transmit();
    void receive();

    TransceiverProxy operator()(std::string cid, std::string id);

    std::function<double()> get_double(std::string cid, std::string id);
    std::function<arma::mat()> get_mat(std::string cid, std::string id);
    std::function<transmit_channel*()> get_gpsr(std::string cid, std::string id);
    std::function<refactor_downlink_packet_t()> get_downlink(std::string cid, std::string id);
    std::function<refactor_uplink_packet_t()> get_uplink(std::string cid, std::string id);

 private:
    TCDevice dev;
    TrickErrorHndlr   err_hndlr;

    std::map<std::string, std::function<double()>> data_double_out;
    std::map<std::string, std::function<arma::mat()>> data_mat_out;
    std::map<std::string, std::function<transmit_channel*()>> data_gpsr_out;
    std::map<std::string, std::function<refactor_downlink_packet_t()>> data_downlink_out;
    std::map<std::string, std::function<refactor_uplink_packet_t()>> data_uplink_out;

    std::map<std::string, double> data_double_in;
    std::map<std::string, arma::mat> data_mat_in;
    std::map<std::string, transmit_channel*> data_gpsr_in;
    std::map<std::string, refactor_downlink_packet_t> data_downlink_in;
    std::map<std::string, refactor_uplink_packet_t> data_uplink_in;
};

class TransceiverProxy{
    Transceiver *transceiver;
    std::string cid;
    std::string id;

 public:
    TransceiverProxy(Transceiver *trans , std::string cid, std::string id)
        :   transceiver(trans), cid(cid), id(id) {
    }
#ifndef TRICK_ICG
#ifndef SWIG
    operator std::function<double()> () {
        return transceiver->get_double(cid, id);
    }

    operator std::function<bool()> () {
        return transceiver->get_double(cid, id);
    }

    operator std::function<int()> () {
        return transceiver->get_double(cid, id);
    }

    operator std::function<arma::mat()> () {
        return transceiver->get_mat(cid, id);
    }

    operator std::function<arma::vec3()> () {
        return transceiver->get_mat(cid, id);
    }

    operator std::function<arma::mat33()> () {
        return transceiver->get_mat(cid, id);
    }

    operator std::function<arma::vec2()> () {
        return transceiver->get_mat(cid, id);
    }

    operator std::function<transmit_channel*()> () {
        return transceiver->get_gpsr(cid, id);
    }

    operator std::function<refactor_downlink_packet_t()> () {
        return transceiver->get_downlink(cid, id);
    }

    operator std::function<refactor_uplink_packet_t()> () {
        return transceiver->get_uplink(cid, id);
    }
#endif
#endif
};

#endif  // __TRANSCEIVER_HH__
