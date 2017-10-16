#include "ezio.hh"

extern "C" {
    #include "driver/EasyCAT.h"
    #include "driver/rpi/gpio.h"
    #include "driver/rpi/spi.h"
    #include "driver/console.h"
    #include <bcm2835.h>
}

#ifndef PPS_GPIO
#define PPS_GPIO 27
#endif

#ifndef SYNC_GPIO
#define SYNC_GPIO 4
#endif

void initialization() {
    std::cout << "Data-Bridge Initialization" << std::endl;

    console_init();

    // gpio init
    gpio_init();

    // spi init
    if (spi_init() < 0) {
        fprintf(stderr, "Spi init failed\n");
        exit(-1);
    }

    // EasyCAT init
    if (Init() != 0) {
        fprintf(stderr, "EasyCAT init failed\n");
        spi_exit();
        exit(-1);
    }

    // config PPS output
    gpio_config_input(PPS_GPIO);  // must set before output
    gpio_config_output(PPS_GPIO);
    gpio_clr(PPS_GPIO);

    // config Sync input, bcm2835_init() already called by spi
    // set to input mode with pulldown
    bcm2835_gpio_fsel(SYNC_GPIO, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(SYNC_GPIO, BCM2835_GPIO_PUD_DOWN);
    bcm2835_gpio_set_eds(SYNC_GPIO);
}

void clean_up() {
    // cleanup Sync edge detection
    bcm2835_gpio_clr_aren(SYNC_GPIO);
    bcm2835_gpio_set_eds(SYNC_GPIO);

    // cleanup spi
    spi_exit();
}

void generate_pps(uint32_t width) {
    // PPS pulse
    gpio_set(PPS_GPIO);
    usleep(width);
    gpio_clr(PPS_GPIO);
}

void load_data(SDT_INTERFACE_t in, Aux_send_data_t aux_in) {
    char buffer_out[sizeof(SDT_INTERFACE_t) + sizeof(Aux_send_data_t)];
    char buffer_in[sizeof(Aux_receive_command_t)];

    memcpy(buffer_out, &in);
    memcpy(buffer_out + sizeof(SDT_INTERFACE_t), &aux_in);

    Maintask(buffer_out, sizeof(buffer_out), buffer_in, sizeof(buffer_in));
}

Aux_receive_command_t wait_fc_command() {
    char buffer_out[sizeof(SDT_INTERFACE_t) + sizeof(Aux_send_data_t)];
    char buffer_in[sizeof(Aux_receive_command_t)];

    // wait for Sync
    bcm2835_gpio_aren(SYNC_GPIO);
    while (!bcm2835_gpio_eds(SYNC_GPIO)) {}
    bcm2835_gpio_clr_aren(SYNC_GPIO);
    bcm2835_gpio_set_eds(SYNC_GPIO);

    Maintask(buffer_out, sizeof(buffer_out), buffer_in, sizeof(buffer_in));

    return reinterpret_cast<Aux_receive_command_t> buffer_in;
}

int main(int argc, char* argv[]) {
    ecio::slave slave("DATA-BRIDGE");
    slave.generate_pps_callback = generate_pps;
    slave.load_data_callback = load_data;
    slave.get_commands_callback = wait_fc_command;

    initialization();

    while (1) {
        slave.process_incoming_command();
    }

    clean_up();

    return 0;
}
