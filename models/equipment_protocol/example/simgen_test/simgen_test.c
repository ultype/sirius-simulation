#include "simgen_remote.h"
#include <unistd.h>
struct simgen_motion_data_t user_data;
struct simgen_eqmt_info_t simgen_eqmt_test;
int main(int argc, char const *argv[]) {

    simgen_equipment_init(&simgen_eqmt_test, &user_data);
    while (1) {
        sleep(1);
        simgen_motion_data_sendto(&simgen_eqmt_test, &user_data);
    }
    return 0;
}