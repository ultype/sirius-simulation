#include "simgen_remote.h"
#include <unistd.h>
int main(int argc, char const *argv[]) {
    struct simgen_motion_data_t user_data;
    struct simgen_eqmt_info_t simgen_eqmt_test;
    simgen_equipment_init(&simgen_eqmt_test, &user_data);
    while (1) {
        sleep(3);
        simgen_motion_data_sendto(&simgen_eqmt_test, &user_data);
    }
    return 0;
}