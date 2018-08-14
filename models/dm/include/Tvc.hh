#ifndef __TVC_HH__
#define __TVC_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the TVC Module On Board)
LIBRARY DEPENDENCY:
      ((../src/Tvc.cpp))
*******************************************************************************/
#include <tuple>
#include <functional>
#include "aux.hh"
#include <armadillo>
#include "global_constants.hh"
#include "icf_trx_ctrl.h"

struct tvc_param_t {
    int valid;
    int16_t pitch_count;
    int16_t yaw_count;
};

class TVC {
    TRICK_INTERFACE(TVC);

 public:
    TVC();
    TVC(const TVC& other);

    TVC& operator=(const TVC& other);

    void initialize();

    void actuate(double int_step, struct icf_ctrlblk_t* C);

    enum TVC_TYPE {
        NO_TVC = 0,
        NO_DYNAMIC_TVC,
        SECON_ORDER_TVC,         // TVC Second order dynamics with rate limiting
        ONLINE_SECOND_ORDER_TVC,  // same as 2nd order but with on-line TVC gain
        S2_TVC,
        S3_TVC
    };

    enum TVC_TYPE get_mtvc();
    void set_mtvc(enum TVC_TYPE);

    double get_gtvc();
    void set_gtvc(double);

    void set_tvclimx(double in);
    void set_dtvclimx(double in);
    void set_wntvc(double in);
    void set_zettvc(double in);
    void set_factgtvc(double in);
    void set_parm(double in);
    void set_S2_TVC();
    void set_S3_TVC();
    void set_s2_tvc_acc_lim(double in);
    void set_s3_tvc_acc_lim(double in);
    void set_s2_tvc_d(double in);
    void set_s3_tvc_d(double in);
    void set_S2_reference_p(double in);
    void set_S3_reference_p(double in);


    double get_parm();

    double get_s2_act1_rate();
    double get_s2_act2_rate();
    double get_s2_act3_rate();
    double get_s2_act4_rate();

    double get_s2_act1_y2_saturation();
    double get_s2_act2_y2_saturation();
    double get_s2_act3_y2_saturation();
    double get_s2_act4_y2_saturation();

    double get_s2_act1_acc();
    double get_s2_act2_acc();
    double get_s2_act3_acc();
    double get_s2_act4_acc();

    std::function<double()> grab_delrcx;
    std::function<double()> grab_delecx;
    std::function<double()> grab_theta_a_cmd;
    std::function<double()> grab_theta_b_cmd;
    std::function<double()> grab_theta_c_cmd;
    std::function<double()> grab_theta_d_cmd;
    std::function<double()> grab_pdynmc;
    std::function<arma::vec3()> grab_xcg;
    std::function<double()> grab_thrust;
    std::function<double()> grab_alphax;
    std::function<arma::mat33()> grab_TBI;
    std::function<arma::vec3()> grab_SBII;


    arma::vec3 get_FPB();
    arma::vec3 get_FMPB();
    arma::vec6 get_Q_TVC();
    arma::vec3 get_lx();

    void set_s2_tau1(double in);
    void set_s2_tau2(double in);
    void set_s2_tau3(double in);
    void set_s2_tau4(double in);
    void set_s2_ratelim(double in);
    void set_s2_tvclim(double in);
    void set_s3_tau1(double in);
    void set_s3_tau2(double in);
    void set_s3_tau3(double in);
    void set_s3_tau4(double in);
    void set_s3_ratelim(double in);
    void set_s3_tvclim(double in);

 private:
    /* Internal Getter */

    /* Internal Initializers */
    void default_data();

    /* Internal Propagator / Calculators */
    // returning: eta, zet,
    std::tuple<double, double> tvc_scnd(double etac, double zetc, double int_step);

    arma::vec3 calculate_FPB(double eta, double zet, double thrust);
    arma::vec3 calculate_FMPB(double xcg);
    arma::vec3 calculate_S2_FPB(double theta_a, double theta_b, double theta_c, double theta_d, double thrust);
    arma::vec3 calculate_S2_FMPB(double theta_a, double theta_b, double theta_c, double theta_d, double thrust, double xcg);

    arma::vec3 calculate_S3_FPB(double theta_a, double theta_b, double theta_c, double theta_d, double thrust);
    arma::vec3 calculate_S3_FMPB(double theta_a, double theta_b, double theta_c, double theta_d, double thrust, double xcg);

    arma::mat33 cross_matrix(arma::vec3 in);

    void S2_actuator_1(double command, double int_step);
    void S2_actuator_2(double command, double int_step);
    void S2_actuator_3(double command, double int_step);
    void S2_actuator_4(double command, double int_step);

    void S3_actuator_1(double command, double int_step);
    void S3_actuator_2(double command, double int_step);
    void S3_actuator_3(double command, double int_step);
    void S3_actuator_4(double command, double int_step);

    void calculate_S2_Q(double theta_a, double theta_b, double theta_c, double theta_d);
    void calculate_S3_Q(double theta_a, double theta_b);

    struct tvc_param_t tvc_no1;
    struct tvc_param_t tvc_no2;
    void decode_frame(struct can_frame *pframe);

    /* State */
    enum TVC_TYPE mtvc;            /* *o  (--)      see TVC_TYPE */

    /* Constants */
    double gtvc;        /* *o  (--)    TVC nozzle deflection gain n*/

    double tvclimx;     /* *o  (d)     Nozzle deflection limiter */
    double dtvclimx;    /* *o  (d/s)   Nozzle deflection rate limiter */
    double wntvc;       /* *o  (r/s)   Natural frequency of TVC */
    double zettvc;      /* *o  (--)    Damping of TVC */
    double factgtvc;    /* *o  (--)    Factor for TVC gain */

    /* Propagative Stats */
    double etas;        /* *o  (r)     Pitch nozzle deflection */
    double etasd;       /* *o  (r/s)   Pitch nozzle derivative */

    double zeta;        /* *o  (r)     Yaw nozzle deflection */
    double zetad;       /* *o  (r/s)   Yaw nozzle derivative */

    double detas;       /* *o  (r/s)   Pitch nozzle rate */
    double detasd;      /* *o  (r/s2)  Pitch nozzle rate derivative */

    double dzeta;       /* *o  (r/s)   Yaw nozzle rate */
    double dzetad;      /* *o  (r/s2)  Yaw nozzle rate derivative */

    /* Generating Outputs */
    double parm;        /* *o  (m)     Propulsion moment arm from vehicle nose n*/

    arma::vec FPB;      /* *o  (N)     Thrust force in body axes n*/
    double   _FPB[3];   /* *o  (N)     Thrust force in body axes n*/

    arma::vec FMPB;     /* *o  (N*m)    Thrust moment in body axes */
    double   _FMPB[3];  /* *o  (N*m)    Thrust moment in body axes */

    double etax;        /* *io  (d)     Nozzle pitch deflection */
    double zetx;        /* *io  (d)     Nozzle yaw deflection */

    /* Non-propagating Diagnostic Variables */
    /* These can be deleted, but keep to remain trackable in trick simulator */

    double etacx;       /* *io  (d)     Commanded nozzle pitch deflection */
    double zetcx;       /* *io  (d)     Commanded nozzle yaw deflection */

    double s2_tau1;     /* *io  (--)    Actuator 1 time constant */
    double s2_tau2;     /* *io  (--)    Actuator 2 time constant */
    double s2_tau3;     /* *io  (--)    Actuator 3 time constant */
    double s2_tau4;     /* *io  (--)    Actuator 4 time constant */

    double s2_act1_y1;  /* *io  (r)     Actuator 1 post state */
    double s2_act2_y1;  /* *io  (r)     Actuator 2 post state */
    double s2_act3_y1;  /* *io  (r)     Actuator 3 post state */
    double s2_act4_y1;  /* *io  (r)     Actuator 4 post state */

    double s2_act1_y1_saturation;  /* *io  (r)     Actuator 1 post state before saturation */
    double s2_act2_y1_saturation;  /* *io  (r)     Actuator 2 post state before saturation */
    double s2_act3_y1_saturation;  /* *io  (r)     Actuator 3 post state before saturation */
    double s2_act4_y1_saturation;  /* *io  (r)     Actuator 4 post state before saturation */

    double s2_act1_y2;  /* *io  (r)     Actuator 1 prior state */
    double s2_act2_y2;  /* *io  (r)     Actuator 2 prior state */
    double s2_act3_y2;  /* *io  (r)     Actuator 3 prior state */
    double s2_act4_y2;  /* *io  (r)     Actuator 4 prior state */

    double s2_act1_y2_saturation;  /* *io  (r)     Actuator 1 prior state before saturation */
    double s2_act2_y2_saturation;  /* *io  (r)     Actuator 2 prior state before saturation */
    double s2_act3_y2_saturation;  /* *io  (r)     Actuator 3 prior state before saturation */
    double s2_act4_y2_saturation;  /* *io  (r)     Actuator 4 prior state before saturation */

    double s2_act1_rate;
    double s2_act2_rate;
    double s2_act3_rate;
    double s2_act4_rate;

    double s2_act1_rate_saturation;
    double s2_act2_rate_saturation;
    double s2_act3_rate_saturation;
    double s2_act4_rate_saturation;

    double s2_act1_rate_saturation_old;
    double s2_act2_rate_saturation_old;
    double s2_act3_rate_saturation_old;
    double s2_act4_rate_saturation_old;

    double s2_act1_rate_old;
    double s2_act2_rate_old;
    double s2_act3_rate_old;
    double s2_act4_rate_old;

    double s2_act1_acc;
    double s2_act2_acc;
    double s2_act3_acc;
    double s2_act4_acc;

    double s3_act1_acc;
    double s3_act2_acc;
    double s3_act3_acc;
    double s3_act4_acc;

    double s2_ratelim;  /* *io  (r/s)   Actuator rate limit */
    double s2_tvclim;   /* *io  (r)     Actuator limit */
    double s2_acclim;    /* *o   (r/s2)  Acutator acc limit */

    double s3_tau1;     /* *io  (--)    Actuator 1 time constant */
    double s3_tau2;     /* *io  (--)    Actuator 2 time constant */
    double s3_tau3;     /* *io  (--)    Actuator 3 time constant */
    double s3_tau4;     /* *io  (--)    Actuator 4 time constant */

    double s3_act1_y1;  /* *io  (r)     Actuator 1 post state */
    double s3_act2_y1;  /* *io  (r)     Actuator 2 post state */
    double s3_act3_y1;  /* *io  (r)     Actuator 3 post state */
    double s3_act4_y1;  /* *io  (r)     Actuator 4 post state */

    double s3_act1_y1_saturation;  /* *io  (r)     Actuator 1 post state before saturation */
    double s3_act2_y1_saturation;  /* *io  (r)     Actuator 2 post state before saturation */
    double s3_act3_y1_saturation;  /* *io  (r)     Actuator 3 post state before saturation */
    double s3_act4_y1_saturation;  /* *io  (r)     Actuator 4 post state before saturation */

    double s3_act1_y2;  /* *io  (r)     Actuator 1 prior state */
    double s3_act2_y2;  /* *io  (r)     Actuator 2 prior state */
    double s3_act3_y2;  /* *io  (r)     Actuator 3 prior state */
    double s3_act4_y2;  /* *io  (r)     Actuator 4 prior state */

    double s3_act1_y2_saturation;  /* *io  (r)     Actuator 1 prior state before saturation */
    double s3_act2_y2_saturation;  /* *io  (r)     Actuator 2 prior state before saturation */
    double s3_act3_y2_saturation;  /* *io  (r)     Actuator 3 prior state before saturation */
    double s3_act4_y2_saturation;  /* *io  (r)     Actuator 4 prior state before saturation */

    double s3_act1_rate;
    double s3_act2_rate;
    double s3_act3_rate;
    double s3_act4_rate;

    double s3_act1_rate_saturation;
    double s3_act2_rate_saturation;
    double s3_act3_rate_saturation;
    double s3_act4_rate_saturation;

    double s3_act1_rate_saturation_old;
    double s3_act2_rate_saturation_old;
    double s3_act3_rate_saturation_old;
    double s3_act4_rate_saturation_old;

    double s3_act1_rate_old;
    double s3_act2_rate_old;
    double s3_act3_rate_old;
    double s3_act4_rate_old;

    double s3_ratelim;  /* *io  (r/s)   Actuator rate limit */
    double s3_tvclim;   /* *io  (r)     Actuator limit */
    double s3_acclim;    /* *o   (r/s2)  Acutator acc limit */

    double theta_a_cmd;
    double theta_b_cmd;
    double theta_c_cmd;
    double theta_d_cmd;

    arma::mat TS2_N1_B;
    double _TS2_N1_B[3][3];

    arma::mat TS2_N2_B;
    double _TS2_N2_B[3][3];

    arma::mat TS2_N3_B;
    double _TS2_N3_B[3][3];

    arma::mat TS2_N4_B;
    double _TS2_N4_B[3][3];

    arma::mat TS3_N1_B;
    double _TS3_N1_B[3][3];

    arma::mat TS3_N2_B;
    double _TS3_N2_B[3][3];

    arma::vec r_N1;
    double _r_N1[3];

    arma::vec r_N2;
    double _r_N2[3];

    arma::vec r_N3;
    double _r_N3[3];

    arma::vec r_N4;
    double _r_N4[3];

    arma::vec Q_TVC;
    double _Q_TVC[6];

    arma::vec S2_FPB1;
    double _S2_FPB1[3];

    arma::vec S2_FPB2;
    double _S2_FPB2[3];

    arma::vec S2_FPB3;
    double _S2_FPB3[3];

    arma::vec S2_FPB4;
    double _S2_FPB4[3];

    arma::vec S2_FMPB1;
    double _S2_FMPB1[3];

    arma::vec S2_FMPB2;
    double _S2_FMPB2[3];

    arma::vec S2_FMPB3;
    double _S2_FMPB3[3];

    arma::vec S2_FMPB4;
    double _S2_FMPB4[3];

    arma::vec lx;
    double _lx[3];

    double s2_d;
    double s3_d;
    double s2_reference_p;
    double s3_reference_p;
};

#endif  // __TVC_HH__
