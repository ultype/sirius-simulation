#include "ezio.hh"

#include <iostream>
#include <cstdio>

#include <unistd.h>

ezio::master::master(char* name) {
    memset(reinterpret_cast<void*>(&err_hndlr), '\0', sizeof(TrickErrorHndlr));
    trick_error_init(&err_hndlr, (TrickErrorFuncPtr)NULL,
                     (TrickErrorDataPtr)NULL, TRICK_ERROR_TRIVIAL);

    char buf[32] = "SIRIUS";
    int status = tc_multiconnect(&dev, name, buf, &err_hndlr);
    tc_blockio(&dev, TC_COMM_BLOCKIO);
    if (status != TC_SUCCESS) {
        perror("Error from tc_multiconnect\n");
        exit(255);
    }
}

ezio::master::~master() {
}

void ezio::master::send_pps() {
    uint32_t tmp = SEND_PPS;
    tc_write(&dev, reinterpret_cast<char*>(&tmp), sizeof(uint32_t));

    return;
}

void ezio::master::load_data(SDT_INTERFACE_t in, Aux_send_data_t aux_in) {
    uint32_t tmp = LOAD_DATA;

    tc_write(&dev, reinterpret_cast<char*>(&tmp), sizeof(uint32_t));
    tc_write(&dev, reinterpret_cast<char*>(&in), sizeof(SDT_INTERFACE_t));
    tc_write(&dev, reinterpret_cast<char*>(&aux_in), sizeof(Aux_send_data_t));

    return;
}

Aux_receive_command_t ezio::master::wait_fc_command() {
    uint32_t tmp = WAIT_FC_COMMAND;
    Aux_receive_command_t commands;

    tc_write(&dev, reinterpret_cast<char*>(&tmp), sizeof(uint32_t));
    tc_read(&dev, reinterpret_cast<char*>(&commands), sizeof(Aux_receive_command_t));

    return commands;
}

void ezio::master::set_pps_width(uint32_t us) {
    uint32_t tmp = SET_PPS_WITDTH;

    tc_write(&dev, reinterpret_cast<char*>(&tmp), sizeof(uint32_t));
    tc_write(&dev, reinterpret_cast<char*>(&us), sizeof(uint32_t));

    return;
}

ezio::slave::slave(char* name) : pps_width(10) {
    memset(reinterpret_cast<void*>(&err_hndlr), '\0', sizeof(TrickErrorHndlr));
    trick_error_init(&err_hndlr, (TrickErrorFuncPtr)NULL,
                     (TrickErrorDataPtr)NULL, TRICK_ERROR_TRIVIAL);

    char buf[32] = "SIRIUS";
    int status = tc_multiconnect(&dev, name, buf, &err_hndlr);
    tc_blockio(&dev, TC_COMM_BLOCKIO);
    if (status != TC_SUCCESS) {
        perror("Error from tc_multiconnect\n");
        exit(255);
    }
}

ezio::slave::~slave() {
}

void ezio::slave::process_incoming_command() {
    uint32_t command;
    SDT_INTERFACE_t in;
    Aux_send_data_t aux_in;
    Aux_receive_command_t commands;

    tc_read(&dev, reinterpret_cast<char*>(&command), sizeof(uint32_t));

    switch (command) {
        case SEND_PPS:
            generate_pps_callback(pps_width);
            break;
        case SET_PPS_WITDTH:
            tc_read(&dev, reinterpret_cast<char*>(&pps_width), sizeof(uint32_t));
            break;
        case LOAD_DATA:
            tc_read(&dev, reinterpret_cast<char*>(&in), sizeof(SDT_INTERFACE_t));
            tc_read(&dev, reinterpret_cast<char*>(&aux_in), sizeof(Aux_send_data_t));
            load_data_callback(in, aux_in);
            break;
        case WAIT_FC_COMMAND:
            commands = get_commands_callback();
            tc_write(&dev, reinterpret_cast<char*>(&commands), sizeof(Aux_receive_command_t));
        default:
            perror("Net Command Unknown");
    }
}
