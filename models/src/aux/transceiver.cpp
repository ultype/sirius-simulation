#include "aux/transceiver.hh"
#include "rocket/Ins.hh"
#include "sim_services/include/simtime.h"

#include "trick_utils/comm/include/tc.h"
#include "trick_utils/comm/include/tc_proto.h"

#include <iostream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/stream_buffer.hpp>
#include <boost/iostreams/device/back_inserter.hpp>

#define TR_BUFFER_SIZE 102400

void Transceiver::initialize_connection(char* name){
    memset((void *)&err_hndlr,'\0',sizeof(TrickErrorHndlr));
    trick_error_init(&err_hndlr, (TrickErrorFuncPtr)NULL,
                     (TrickErrorDataPtr)NULL, TRICK_ERROR_TRIVIAL);

    int status = tc_multiconnect(&dev , name , "SIRIUS" , &err_hndlr);
    tc_blockio(&dev, TC_COMM_BLOCKIO);
    if (status != TC_SUCCESS) {
        perror("Error from tc_multiconnect\n");
        exit(255);
    }
}

void Transceiver::begin_transmit(){
    data_out.clear();
}

void Transceiver::register_for_transmit_double(std::function<double()> in){
    data_out.push_back(in());
}

void Transceiver::register_for_transmit_vec3(std::function<arma::vec3()> in){
    arma::vec3 data = in();
    for(int i = 0; i < 3; i++){
        data_out.push_back(data(i));
    }
}

void Transceiver::register_for_transmit_mat33(std::function<arma::mat33()> in){
    arma::mat33 data = in();
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            data_out.push_back(data(i, j));
        }
    }
}

void Transceiver::transmit(){
    uint32_t size = sizeof(double) * data_out.size();
    if (tc_isValid(&dev)) {
        tc_write(&dev, (char*)&size, sizeof(uint32_t));
    }

    if (tc_isValid(&dev)) {
        auto out = [this](const double& n) { tc_write(&dev, (char*)&n, sizeof(double)); };
        std::for_each(data_out.begin(), data_out.end(), out);
    }
}

uint32_t Transceiver::begin_receive(){
    uint32_t size;
    if (tc_isValid(&dev)) {
        tc_read(&dev, (char*)&size, sizeof(uint32_t));
    }

    data_in_tag.clear();
    data_in_double.clear();
    data_in_vec3.clear();
    data_in_mat33.clear();

    return size;
}

std::function<double()> Transceiver::register_to_receive_double(std::string name){
    double in;
    if (tc_isValid(&dev)) {
        if(data_in_tag.find(name) == data_in_tag.end()){
            tc_read(&dev, (char*)&in, sizeof(double));
            data_in_double[name] = in;
            data_in_tag.insert(name);
        }
    }

    return std::bind(&Transceiver::get_double, this, name);
}

std::function<arma::vec3()> Transceiver::register_to_receive_vec3(std::string name){
    arma::vec3 in;
    if (tc_isValid(&dev)) {
        if(data_in_tag.find(name) == data_in_tag.end()){
            for(int i = 0; i < 3; i++){
                tc_read(&dev, (char*)&(in(i)), sizeof(double));
            }
            data_in_vec3[name] = in;
            data_in_tag.insert(name);
        }
    }

    return std::bind(&Transceiver::get_vec3, this, name);
}

std::function<arma::mat33()> Transceiver::register_to_receive_mat33(std::string name){
    arma::mat33 in;
    if (tc_isValid(&dev)) {
        if(data_in_tag.find(name) == data_in_tag.end()){
            for(int i = 0; i < 3; i++){
                for(int j = 0; j < 3; j++){
                    tc_read(&dev, (char*)&(in(i, j)), sizeof(double));
                }
            }
            data_in_mat33[name] = in;
            data_in_tag.insert(name);
        }
    }

    return std::bind(&Transceiver::get_mat33, this, name);
}

double Transceiver::get_double(std::string id){
    if(data_in_tag.find(id) != data_in_tag.end()){
        return data_in_double[id];
    }else{
        throw std::invalid_argument(id);
    }
    return 0;
}

arma::vec3 Transceiver::get_vec3(std::string id){
    if(data_in_tag.find(id) != data_in_tag.end()){
        return data_in_vec3[id];
    }else{
        throw std::invalid_argument(id);
    }
    return arma::vec3(arma::fill::zeros);
}

arma::mat33 Transceiver::get_mat33(std::string id){
    if(data_in_tag.find(id) != data_in_tag.end()){
        return data_in_mat33[id];
    }else{
        throw std::invalid_argument(id);
    }
    return arma::mat33(arma::fill::zeros);
}

