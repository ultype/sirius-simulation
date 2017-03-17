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

void Transceiver::transmit(INS& ins){
    std::string serial_str;
    boost::iostreams::back_insert_device<std::string> inserter(serial_str);
    boost::iostreams::stream<boost::iostreams::back_insert_device<std::string> > s(inserter);
    boost::archive::binary_oarchive oa(s);

    oa << ins;

    s.flush();

    uint32_t len = serial_str.length();
    if (tc_isValid(&dev)) {
        tc_write(&dev, (char*)&len, sizeof(uint32_t));
    }

    if (tc_isValid(&dev)) {
        tc_write(&dev, (char*)serial_str.data(), serial_str.length());
    }
}

void Transceiver::receive(INS& ins){
    uint32_t len;
    if (tc_isValid(&dev)) {
        tc_read(&dev, (char*)&len, sizeof(uint32_t));
    }

    char* buf = (char*)malloc(len + 1);
    if(!buf){
        perror("allocating receive buffer\n");
        exit(255);
    }

    if (tc_isValid(&dev)) {
        tc_read(&dev, buf, len);
    }

    boost::iostreams::basic_array_source<char> device(buf, len);
    boost::iostreams::stream<boost::iostreams::basic_array_source<char> > s(device);
    boost::archive::binary_iarchive ia(s);
    ia >> ins;

    free(buf);
}
