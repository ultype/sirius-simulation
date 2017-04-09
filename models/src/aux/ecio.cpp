#include "aux/ecio.hh"
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

#define LOCAL_PORT 23138

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
}

Ecio::~Ecio(){
    //cleanup Sync edge detection
    bcm2835_gpio_clr_aren(SYNC_GPIO);
    bcm2835_gpio_set_eds(SYNC_GPIO);

    //cleanup spi
    spi_exit();
}

void Ecio::socket_bind_connection(){
    portno = LOCAL_PORT;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)  puts("ERROR opening socket");
    server = gethostbyname("192.168.1.1");
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
            (char *)&serv_addr.sin_addr.s_addr,
            server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        puts("ERROR connecting");
}

void Ecio::prepare_ethercat_packet(){
    net_data_t data;

    vec3_to_array(grab_computed_FSPB(), data.accel_FSPCB);
    vec3_to_array(grab_error_of_computed_FSPB(), data.accel_EFSPB);
    data.aero_value[0] = grab_gymax();
    data.aero_value[1] = grab_dyb();
    data.aero_value[2] = grab_dnb();
    data.aero_value[3] = grab_dnr();
    data.aero_value[4] = grab_dndr();
    data.aero_value[5] = grab_dla();
    data.aero_value[6] = grab_dma();
    data.aero_value[7] = grab_dmq();
    data.aero_value[8] = grab_dmde();
    data.aero_value[9] = grab_dnd();
    data.aero_value[10] = grab_dlde();
    data.env_pdynmc = grab_pdynmc();
    vec3_to_array(grab_computed_WBIB(), data.gyro_WBICB);
    vec3_to_array(grab_error_of_computed_WBIB(), data.gyro_EWBIB);
    vec3_to_array(grab_SXH(), data.gpsr_SXH);
    vec3_to_array(grab_VXH(), data.gpsr_VXH);
    data.gpsr_gps_update = grab_gps_update();
    mat33_to_array(grab_TBI(), data.kinematics_TBI);
    vec3_to_array(grab_SBII(), data.newton_SBII);
    vec3_to_array(grab_VBII(), data.newton_VBII);
    data.propulsion_thrust_state = (enum Propulsion::THRUST_TYPE)grab_mprop();
    data.propulsion_fmassr = grab_fmassr();
    data.newton_dvbe = grab_dvbe();
    data.newton_dbi = grab_dbi();
    data.newton_dvbi = grab_dvbi();
    data.newton_thtvdx = grab_thtvdx();
    data.gyro_qqcx = grab_qqcx();
    data.gyro_ppcx = grab_ppcx();
    data.gyro_rrcx = grab_rrcx();

    int send_size = send(sockfd, &data, sizeof(net_data_t), 0);
}

void Ecio::send_pps(){
    //PPS pulse
    gpio_set(PPS_GPIO);
    usleep(50);
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
    int read_size;
    int total = 0;
    uint8_t *buffer = (uint8_t *)&data_in;

    while( total != sizeof(net_data_t) && (read_size = recv(sockfd , buffer, sizeof(net_data_t), 0)) > 0 ){
        buffer += read_size;
        total += read_size;
    }
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
    return data_in.rcsfc_rcs_mode;
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
