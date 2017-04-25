#include "aux/transceiver.hh"
#include "sim_services/include/simtime.h"

#include "trick_utils/comm/include/tc.h"
#include "trick_utils/comm/include/tc_proto.h"

#include <iostream>

#define TR_BUFFER_SIZE 102400

enum packet_type{
    DOUBLE,
    MAT,
    STRUCT
};

struct __attribute__ ((__packed__)) generic_header{
    unsigned int type : 1;
    unsigned int name_length : 7;
};

struct __attribute__ ((__packed__)) mat_header{
    long long unsigned int x : 4;
    long long unsigned int y : 4;
};

void Transceiver::initialize_connection(char* name){
    memset((void *)&err_hndlr,'\0',sizeof(TrickErrorHndlr));
    trick_error_init(&err_hndlr, (TrickErrorFuncPtr)NULL,
                     (TrickErrorDataPtr)NULL, TRICK_ERROR_TRIVIAL);

    int status = tc_multiconnect(&dev , name , (char*)"SIRIUS" , &err_hndlr);
    tc_blockio(&dev, TC_COMM_BLOCKIO);
    if (status != TC_SUCCESS) {
        perror("Error from tc_multiconnect\n");
        exit(255);
    }
}

void Transceiver::register_for_transmit(std::string cid, std::string id, std::function<double()> in){
    std::string tid = cid + "." + id;
    if(tid.length() > 127) throw std::out_of_range("ID too long");
    data_double_out[tid] = in;
}

void Transceiver::register_for_transmit(std::string cid, std::string id, std::function<arma::mat()> in){
    std::string tid = cid + "." + id;
    if(tid.length() > 127) throw std::out_of_range("ID too long");
    data_mat_out[tid] = in;
}

void Transceiver::register_for_transmit(std::string cid, std::string id, std::function<transmit_channel*()> in){
    std::string tid = cid + "." +id;
    if(tid.length() > 127) throw std::out_of_range("ID too long");
    data_gpsr_out[tid] = in;
}

void Transceiver::transmit(){
    uint32_t size = data_double_out.size() + data_mat_out.size() + data_gpsr_out.size();
    if (tc_isValid(&dev)) {
        tc_write(&dev, (char*)&size, sizeof(uint32_t));
    }

    for (auto it = data_double_out.begin(); it != data_double_out.end(); ++it) {
        struct generic_header gh = { .type = DOUBLE, .name_length = (unsigned int)it->first.length() };
        double tmp;
        tc_write(&dev, (char*)&gh, sizeof(gh));
        tc_write(&dev, (char*)it->first.c_str(), gh.name_length);
        tmp = it->second();
        tc_write(&dev, (char*)&tmp, sizeof(double));
    }

    for (auto it = data_mat_out.begin(); it != data_mat_out.end(); ++it) {
        struct generic_header gh = { .type = MAT, .name_length = (unsigned int)it->first.length() };
        struct mat_header mh = { .x = it->second().n_rows, .y = it->second().n_cols };
        tc_write(&dev, (char*)&gh, sizeof(gh));
        tc_write(&dev, (char*)&mh, sizeof(mh));
        tc_write(&dev, (char*)it->first.c_str(), gh.name_length);

        auto out_fn = [this](double& val) { tc_write(&dev, (char*)&val, sizeof(double)); };
        it->second().for_each(out_fn);
    }

    for (auto it = data_gpsr_out.begin(); it != data_gpsr_out.end(); ++it) {
        struct generic_header gh = { .type = STRUCT, .name_length = (unsigned int)it->first.length() };
        transmit_channel * tmp;
        tc_write(&dev, (char*)&gh, sizeof(gh));
        tc_write(&dev, (char*)it->first.c_str(), gh.name_length);
        tmp = it->second();
        tc_write(&dev, (char*)&tmp, sizeof(tmp));
    }
}

void Transceiver::receive(){
    uint32_t size;
    if (tc_isValid(&dev)) {
        tc_read(&dev, (char*)&size, sizeof(uint32_t));
    }

    for(int i = 0; i < size; i++){
        char *buf;
        struct generic_header gh;
        struct mat_header mh;
        tc_read(&dev, (char*)&gh, sizeof(gh));
        switch(gh.type){
            case DOUBLE:
                {
                    buf = new char[gh.name_length + 1];
                    tc_read(&dev, buf, gh.name_length);
                    buf[gh.name_length] = '\0';
                    std::string name(buf);
                    delete[] buf;

                    double in;
                    tc_read(&dev, (char*)&in, sizeof(double));
                    data_double_in[name] = in;
                }
                break;
            case MAT:
                {
                    tc_read(&dev, (char*)&mh, sizeof(mh));
                    buf = new char[gh.name_length + 1];
                    tc_read(&dev, buf, gh.name_length);
                    buf[gh.name_length] = '\0';
                    std::string name(buf);
                    delete[] buf;

                    arma::mat in(mh.x, mh.y);
                    auto in_fn = [this](double& val) { tc_read(&dev, (char*)&val, sizeof(double)); };
                    in.for_each(in_fn);
                    if(!data_mat_in.count(name)){
                        data_mat_in.insert(std::make_pair(name, in));
                    }else{
                        data_mat_in[name] = in;
                    }
                }
                break;
            case STRUCT:
                {
                    buf = new char[gh.name_length];
                    tc_read(&dev, buf, gh.name_length);
                    buf[gh.name_length] = '\0';
                    std::string name(buf);
                    delete[] buf;

                    transmit_channel * in;
                    tc_read(&dev, (char*)&in, sizeof(in));
                    data_gpsr_in[name] = in;
                }
                break;    
            default:
                throw std::runtime_error("Received Data Corrupted");
                break;
        }
    }

    return;
}

TransceiverProxy Transceiver::operator ()(std::string cid, std::string id){
    return TransceiverProxy(this, cid, id);
}

std::function<double()> Transceiver::get_double(std::string cid, std::string id){
    return [this, cid, id](){ return data_double_in[cid + "." + id]; };

}

std::function<arma::mat()> Transceiver::get_mat(std::string cid, std::string id){
    return [this, cid, id](){ return data_mat_in[cid + "." + id]; };
}

std::function<transmit_channel *()> Transceiver::get_gpsr(std::string cid, std::string id){
    return [this, cid, id](){ return data_gpsr_in[cid + "." + id]; };
}