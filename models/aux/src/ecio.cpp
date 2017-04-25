#include "ecio.hh"
#include <iostream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>

extern "C"{
    #include "driver/EasyCAT.h"
    #include "driver/rpi/gpio.h"
    #include "driver/rpi/spi.h"
    #include "driver/console.h"
    #include <bcm2835.h>
}

#define PPS_GPIO 27
#define SYNC_GPIO 4

Ecio::Ecio(){
    std::cout << "EtherCAT IO Realy Initialization" << std::endl;

    console_init();

    //gpio init
    gpio_init();

    //spi init
    if(spi_init() < 0){
        fprintf(stderr,"Spi init failed\n");
        exit(-1);
    }

    //EasyCAT init
    if(Init() != 0){
        fprintf(stderr,"EasyCAT init failed\n");
        spi_exit();
        exit(-1);
    }

    //config PPS output
    gpio_config_input(PPS_GPIO);  //must set before output
    gpio_config_output(PPS_GPIO);
    gpio_clr(PPS_GPIO);

    //config Sync input, bcm2835_init() already called by spi
    //set to input mode with pulldown
    bcm2835_gpio_fsel(SYNC_GPIO,BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(SYNC_GPIO, BCM2835_GPIO_PUD_DOWN);
    bcm2835_gpio_set_eds(SYNC_GPIO);
}

Ecio::~Ecio(){
    //cleanup Sync edge detection
    bcm2835_gpio_clr_aren(SYNC_GPIO);
    bcm2835_gpio_set_eds(SYNC_GPIO);

    //cleanup spi
    spi_exit();
}

void Ecio::prepare_ethercat_packet(){

    vec3_to_array(grab_computed_FSPB(), data_out.accel_FSPCB);
    vec3_to_array(grab_error_of_computed_FSPB(), data_out.accel_EFSPB);
    data_out.aero_value[0] = grab_gymax();
    data_out.aero_value[1] = grab_dyb();
    data_out.aero_value[2] = grab_dnb();
    data_out.aero_value[3] = grab_dnr();
    data_out.aero_value[4] = grab_dndr();
    data_out.aero_value[5] = grab_dla();
    data_out.aero_value[6] = grab_dma();
    data_out.aero_value[7] = grab_dmq();
    data_out.aero_value[8] = grab_dmde();
    data_out.aero_value[9] = grab_dnd();
    data_out.aero_value[10] = grab_dlde();
    data_out.env_pdynmc = grab_pdynmc();
    vec3_to_array(grab_computed_WBIB(), data_out.gyro_WBICB);
    vec3_to_array(grab_error_of_computed_WBIB(), data_out.gyro_EWBIB);
    vec3_to_array(grab_SXH(), data_out.gpsr_SXH);
    vec3_to_array(grab_VXH(), data_out.gpsr_VXH);
    data_out.gpsr_gps_update = grab_gps_update();
    mat33_to_array(grab_TBI(), data_out.kinematics_TBI);
    vec3_to_array(grab_SBII(), data_out.newton_SBII);
    vec3_to_array(grab_VBII(), data_out.newton_VBII);
    data_out.propulsion_thrust_state = (enum Propulsion::THRUST_TYPE)grab_mprop();
    data_out.propulsion_fmassr = grab_fmassr();
    data_out.newton_dvbe = grab_dvbe();
    data_out.newton_dbi = grab_dbi();
    data_out.newton_dvbi = grab_dvbi();
    data_out.newton_thtvdx = grab_thtvdx();
    data_out.gyro_qqcx = grab_qqcx();
    data_out.gyro_ppcx = grab_ppcx();
    data_out.gyro_rrcx = grab_rrcx();

    Maintask(data_out,sizeof(data_out),data_in,sizeof(data_in));
}

void Ecio::send_pps(){
    //PPS pulse
    gpio_set(PPS_GPIO);
    usleep(1);
    gpio_clr(PPS_GPIO);
}

void Ecio::wait_ready(){
    //wait for Sync
    bcm2835_gpio_aren(SYNC_GPIO);
    while(!bcm2835_gpio_eds(SYNC_GPIO)){}
    bcm2835_gpio_clr_aren(SYNC_GPIO);
    bcm2835_gpio_set_eds(SYNC_GPIO);
}

void Ecio::receive_fc_data(){
    Maintask(data_out,sizeof(data_out),data_in,sizeof(data_in));
}

void Ecio::vec3_to_array(arma::vec3 in, double array[3]){
    for(int i = 0; i < 3; i++)
        array[i] = in(i);
}

void Ecio::mat33_to_array(arma::mat33 in, double array[3][3]){
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            array[i][j] = in(i, j);
}

double Ecio::get_delrcx(){
    return data_in.control_delrcx;
}

double Ecio::get_delecx(){
    return data_in.control_delecx;
}

bool Ecio::isEnabled(){
    return (bool)data_in.rcs_isEnable;
}

enum RCS_FC::RCS_MODE Ecio::get_rcs_mode(){
    return (enum RCS_FC::RCS_MODE)data_in.rcsfc_rcs_mode;
}

double Ecio::get_e_roll(){
    return data_in.rcsfc_e_roll;
}

double Ecio::get_e_pitch(){
    return data_in.rcsfc_e_pitch;
}

double Ecio::get_e_yaw(){
    return data_in.rcsfc_e_yaw;
}

arma::vec3 Ecio::get_SBIIC(){
    return arma::vec3(data_in.ins_SBIIC);
}

arma::vec3 Ecio::get_VBIIC(){
    return arma::vec3(data_in.ins_VBIIC);
}

arma::vec3 Ecio::get_WBICI(){
    return arma::vec3(data_in.ins_WBICI);
}

int Ecio::get_clear_gps_flag(){
    return data_in.gpsr_flag_for_clear_flag;
}

int Ecio::get_no_thrust_flag(){
    return data_in.propulsion_flag_for_set_no_thrust;
}

int Ecio::get_ltg_thrust_flag(){
    return data_in.propulsion_flag_for_set_ltg_thrust;
}
