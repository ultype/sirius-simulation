#ifndef __ecio_HH__
#define __ecio_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (EtherCAT IO Realy)
LIBRARY DEPENDENCY:
      ((../src/ecio.cpp)
       (../src/driver/EasyCAT.c)
       (../src/driver/rpi/gpio.c)
       (../src/driver/rpi/spi.c)
       )
*******************************************************************************/
#include <functional>
#include <armadillo>

#include "transceiver.hh"
#include "RcsFc.hh"
#include "Propulsion.hh"

class Ecio {
 public:
    Ecio();
    virtual ~Ecio();

    void prepare_ethercat_packet();
    void send_pps();
    void wait_ready();
    void receive_fc_data();

    std::function<int()> grab_mprop;
    std::function<double()> grab_dvbe;
    std::function<double()> grab_gymax;
    std::function<double()> grab_dyb;
    std::function<double()> grab_dnb;
    std::function<double()> grab_dnr;
    std::function<double()> grab_dndr;
    std::function<double()> grab_dla;
    std::function<double()> grab_dma;
    std::function<double()> grab_dmq;
    std::function<double()> grab_dmde;
    std::function<double()> grab_dnd;
    std::function<double()> grab_dlde;
    std::function<double()> grab_pdynmc;
    std::function<arma::vec3()> grab_computed_WBIB;
    std::function<arma::vec3()> grab_error_of_computed_WBIB;
    std::function<arma::vec3()> grab_computed_FSPB;
    std::function<arma::vec3()> grab_error_of_computed_FSPB;
    std::function<arma::vec3()> grab_SBII;
    std::function<arma::vec3()> grab_VBII;
    std::function<arma::mat33()> grab_TBI;
    std::function<arma::vec3()> grab_SXH;
    std::function<arma::vec3()> grab_VXH;
    std::function<int()> grab_gps_update;
    std::function<double()> grab_dbi;
    std::function<double()> grab_dvbi;
    std::function<double()> grab_thtvdx;
    std::function<double()> grab_fmassr;
    std::function<double()> grab_qqcx;
    std::function<double()> grab_ppcx;
    std::function<double()> grab_rrcx;

    double get_delrcx();
    double get_delecx();
    bool isEnabled();
    enum RCS_FC::RCS_MODE get_rcs_mode();
    double get_e_roll();
    double get_e_pitch();
    double get_e_yaw();
    arma::vec3 get_SBIIC();
    arma::vec3 get_VBIIC();
    arma::vec3 get_WBICI();

    int get_clear_gps_flag();
    int get_no_thrust_flag();
    int get_ltg_thrust_flag();

 private:
    typedef struct __attribute__((__packed__)) {
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
    } net_data_tx_t;

    typedef struct __attribute__((__packed__)) {
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
    } net_data_rx_t;

    net_data_tx_t dataa_out;
    net_data_rx_t data_in;

    void vec3_to_array(arma::vec3 in, double array[3]);
    void mat33_to_array(arma::mat33 in, double array[3][3]);
};

#endif  // __ecio_HH__
