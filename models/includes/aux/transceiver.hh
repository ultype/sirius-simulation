#ifndef __TRANSCEIVER_HH__
#define __TRANSCEIVER_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Master-Slave Transmission)
LIBRARY DEPENDENCY:
      ((../../src/aux/transceiver.cpp))
*******************************************************************************/
#include "trick_utils/comm/include/tc.h"
#include "trick_utils/comm/include/tc_proto.h"

#include <armadillo>

#include <functional>
#include <map>
#include <set>
#include <list>

class Transceiver;
class TransceiverProxy;

class Transceiver {
    public:
        Transceiver() {};

        void initialize_connection(char* name);

        void register_for_transmit(std::string cid, std::string id, std::function<double()> in);
        void register_for_transmit(std::string cid, std::string id, std::function<arma::mat()> in);

        void transmit();
        void receive();

        TransceiverProxy operator ()(std::string cid, std::string id);

        std::function<double()> get_double(std::string cid, std::string id);
        std::function<arma::mat()> get_mat(std::string cid, std::string id);

    private:
        TCDevice dev;
        TrickErrorHndlr   err_hndlr;

        std::map<std::string, std::function<double()>> data_double_out;
        std::map<std::string, std::function<arma::mat()>> data_mat_out;

        std::map<std::string, double> data_double_in;
        std::map<std::string, arma::mat> data_mat_in;
};

class TransceiverProxy{
    Transceiver *transceiver;
    std::string cid;
    std::string id;
public:
    TransceiverProxy(Transceiver *trans , std::string cid, std::string id)
        :   transceiver(trans), cid(cid), id(id)
    {
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
#endif
#endif
};

#endif  // __TRANSCEIVER_HH__
