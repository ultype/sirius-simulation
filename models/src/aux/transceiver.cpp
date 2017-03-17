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

void Transceiver::transmit(void* ptr, uint32_t size){
    if (tc_isValid(&dev)) {
        tc_write(&dev, (char*)&size, sizeof(uint32_t));
    }

    if (tc_isValid(&dev)) {
        tc_write(&dev, (char*)ptr, size);
    }
}

uint32_t Transceiver::receive_size(){
    uint32_t size;
    if (tc_isValid(&dev)) {
        tc_read(&dev, (char*)&size, sizeof(uint32_t));
    }

    return size;
}

void Transceiver::receive_data(void* ptr, uint32_t size){
    if (tc_isValid(&dev)) {
        tc_read(&dev, (char*)ptr, size);
    }
}
