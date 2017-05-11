#ifndef __EZIO_HH__
#define __EZIO_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Easy IO (EZIO), simulation data trasnport of standard SDT data format)
LIBRARY DEPENDENCY:
      ((../src/ezio.cpp)
       (../src/driver/EasyCAT.c)
       (../src/driver/rpi/gpio.c)
       (../src/driver/rpi/spi.c)
       )
*******************************************************************************/
#include <functional>

#include "sdt/sdt_interface.hh"

namespace ezio {

struct Aux_send_data_t {
    double accel_FSPCB[3];
    double accel_EFSPB[3];
    double aero_value[11];
    double env_pdynmc;
    double gyro_WBICB[3];
    double gyro_EWBIB[3];
    int gpsr_gps_update;
    double gpsr_SXH[3];
    double gpsr_VXH[3];
    double kinematics_TBI[3][3];
    double newton_SBII[3];
    double newton_VBII[3];
    double newton_TDI[3][3];
    int propulsion_thrust_state;   /* *o (--)     Propulsion mode, See THRUST TYPE*/
    double propulsion_fmassr;              /* *o (kg)     Remaining fuel mass*/
    double newton_dvbe;
    double newton_dbi;
    double newton_dvbi;
    double newton_thtvdx;
    double gyro_qqcx;
    double gyro_ppcx;
    double gyro_rrcx;
} __attribute__((__packed__));

struct Aux_receive_command_t {
    double control_delrcx;
    double control_delecx;
    int rcs_isEnable;
    int rcsfc_rcs_mode;  /* *o  (--)   Attitude control, see RCS_MODE */
    double  rcsfc_e_roll;          /* *o  (--)   Roll error signal */
    double  rcsfc_e_pitch;         /* *o  (--)   Pitch error signal */
    double  rcsfc_e_yaw;           /* *o  (--)   Yaw error signal */
    double ins_SBIIC[3];
    double ins_VBIIC[3];
    double ins_WBICI[3];
    int gpsr_flag_for_clear_flag;
    int propulsion_flag_for_set_no_thrust;
    int propulsion_flag_for_set_ltg_thrust;
} __attribute__((__packed__));

enum COMMAND{
    SEND_PPS,
    LOAD_DATA,
    SET_PPS_WITDTH,
    WAIT_FC_COMMAND,
    COMMAND_NUM
};

class master {
 public:
    master(char* name);
    virtual ~master();

    void send_pps();
    void load_data(SDT_INTERFACE_t in, Aux_send_data_t aux_in);

    Aux_receive_command_t wait_fc_command();

    void set_pps_width(uint32_t us);
 private:
    TCDevice dev;
    TrickErrorHndlr   err_hndlr;
};

class slave {
 public:
    slave(char* name);
    virtual ~slave();

    std::function<void(uint32_t)> generate_pps_callback;
    std::function<void(SDT_INTERFACE_t, Aux_send_data_t)> load_data_callback;
    std::function<Aux_receive_command_t()> get_commands_callback;

    void process_incoming_command();

 private:
    TCDevice dev;
    TrickErrorHndlr   err_hndlr;

    uint32_t pps_width;
};

}  // namespace ezio

#endif  // __EZIO_HH__
