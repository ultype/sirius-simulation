#ifndef __Rocket_Flight_DM_HH__
#define __Rocket_Flight_DM_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the Rocket Flgiht Dynamics Module Variables and Algorithm)
LIBRARY DEPENDENCY:
      ((../src/Rocket_Flight_DM.cpp))
PROGRAMMERS:
      (((Lai Jun Xu) () () () ))
*******************************************************************************/
#include "global_constants.hh"
#include "Environment.hh"
#include <armadillo>
#include "Force.hh"
#include "Propulsion.hh"
#include "Time_management.hh"
#include "aux.hh"
#include "icf_trx_ctrl.h"
#include "simgen_remote.h"
class Environment;
class Propulsion;
class Forces;

class Rocket_Flight_DM {
    TRICK_INTERFACE(Rocket_Flight_DM);

 public:
    Rocket_Flight_DM();
    Rocket_Flight_DM(const Rocket_Flight_DM& other);
    Rocket_Flight_DM& operator=(const Rocket_Flight_DM& other);
    struct icf_ctrlblk_t *dm_icf_info_hook;
    int enqueue_to_simgen_buffer(struct icf_ctrlblk_t* C, double ext_porlation);
    int stand_still_motion_data(struct icf_ctrlblk_t* C, double ext_porlation);
    double get_ppx();
    double get_qqx();
    double get_rrx();
    double get_alppx();
    double get_phipx();
    double get_alphax();
    double get_betax();
    double get_psibdx();
    double get_thtbdx();
    double get_phibdx();
    double get_alt();
    double get_lonx();
    double get_latx();
    double get_dvbe();
    double get_dbi();
    double get_dvbi();
    double get_thtvdx();
    double get_psivdx();
    double get_thtbdx_in(double &cthtbd);

    void initialize();
    void propagate(double int_step);
    void load_location(double lonx, double latx, double alt);
    void load_geodetic_velocity(double alpha0x, double beta0x, double dvbe);
    void load_coning_var(double ang, double w);
    void load_angle(double yaw, double roll, double pitch);
    void load_angular_velocity(double ppx, double qqx, double rrx);
    void update_diagnostic_attributes(double int_step);
    void Interpolation_Extrapolation(double T, double int_step, double ext_porlation);
    void set_reference_point(double rp);

    arma::mat get_TGI();
    arma::mat get_TDE();
    arma::mat get_TBD();
    arma::mat get_TBI();

    arma::vec3 get_FSPB();
    arma::vec3 get_SBII();
    arma::vec3 get_VBII();
    arma::vec3 get_ABII();
    arma::vec3 get_SBEE();
    arma::vec3 get_VBED();
    arma::vec3 get_VBEE();
    arma::vec3 get_CONING();
    arma::vec3 get_WBII();
    arma::vec3 get_WBIB();
    arma::vec3 get_WEII();
    arma::vec3 get_VBAB();
    arma::vec3 get_WBIBD();
    arma::vec3 get_NEXT_ACC();
    arma::vec3 get_SBEE_test();
    arma::vec3 get_VBEE_test();
    arma::vec3 get_ABEE_test();
    double get_dang_slosh_theta();
    double get_ang_slosh_theta();
    double get_dang_slosh_psi();
    double get_ang_slosh_psi();

    double *get_double_SBEE();
    double *get_double_VBEE();
    double *get_double_ABEE();
    double *get_double_JBEE();
    double *get_double_WBEB();
    double *get_double_SBEE_test();
    double *get_double_VBEE_test();
    double *get_double_ABEE_test();
    double get_double_psibd();
    double get_double_thtbd();
    double get_double_phibd();

    unsigned int get_liftoff();
    void set_liftoff(unsigned int in);

    std::function<arma::mat33()> grab_TEI;
    std::function<double()> grab_dvba;
    std::function<arma::vec3()> grab_VAED;
    std::function<arma::vec3()> grab_FMB;
    std::function<arma::mat33()> grab_IBBB;
    std::function<double()> grab_vmass;
    std::function<arma::vec3()> grab_FAPB;
    std::function<arma::vec3()> grab_GRAVG;
    std::function<arma::vec3()> grab_FAP;
    std::function<double()> grab_grav;
    std::function<arma::vec3()> grab_ddrP_1;
    std::function<arma::vec3()> grab_ddang_1;
    std::function<double()> grab_thrust;
    std::function<arma::vec3()> grab_ddrhoC_1;
    std::function<arma::vec3()> grab_rhoC_1;
    std::function<arma::vec3()> grab_xcg_0;
    std::function<double()> grab_ddang_slosh_theta;
    std::function<double()> grab_ddang_slosh_psi;
    std::function<void()> collect_forces_and_propagate;
    std::function<arma::vec6()> grab_Q_TVC;

    struct TX_data {
        double SBEE[3];
        double VBEE[3];
        double ABEE[3];
        double JBEE[3];
        double psibd;
        double thtbd;
        double phibd;
        double WBEB[3];
    } TX_data_forward;

 private:
    Propulsion *propulsion;

    void propagate_position_speed_acceleration(double int_step);
    void propagate_aeroloss(double int_step);
    void propagate_gravityloss(double int_step);
    void propagate_control_loss(double int_step);
    void vibration(double int_step);
    void propagate_TBI(double int_step, arma::vec3 WBIB);
    void propagate_TBI_Q(double int_step, arma::vec3 WBIB);
    void propagate_WBIB(double int_step, arma::vec3 FMB, arma::mat33 IBBB);
    void orbital(arma::vec3 SBII, arma::vec3 VBII, double dbi);
    void build_WEII();
    void aux_calulate(arma::mat33 TEI, arma::mat33 TBI);
    void RK4F(arma::vec3 GRAVG, arma::mat33 TEI, double int_step,
                arma::vec3 &K1, arma::vec3 &K2, arma::vec3 &K3, arma::vec4 &K4,
                double &K5, double &K6, double &K7, double &K8);
    void RK4(arma::vec3 GRAVG, arma::mat33 TEI, double int_step);

    double calculate_alphaix(arma::vec3 VBIB);
    double calculate_betaix(arma::vec3 VBIB);
    double calculate_alppx(arma::vec3 VBAB, double dvba);
    double calculate_phipx(arma::vec3 VBAB);
    double calculate_alphax(arma::vec3 VBAB);
    double calculate_betax(arma::vec3 VBAB, double dvba);

    arma::vec build_VBEB(double _alpha0x, double _beta0x, double _dvbe);
    arma::mat calculate_TBD(double lonx, double latx, double alt);
    arma::vec3 calculate_WBII(arma::mat33 TBI);
    arma::vec3 calculate_fspb(arma::vec3 FAPB, double vmass);
    arma::vec3 calculate_WBEB(arma::mat33 TBI);
    arma::vec3 euler_angle(arma::mat33 TBD);

    arma::mat TBI;      /* *io (--)    Transformation Matrix of body coord wrt inertia coord */
    double _TBI[3][3];  /* *io (--)    Transformation Matrix of body coord wrt inertia coord */

    arma::mat TBID;     /* *io (--)    Transformation Matrix of body coord wrt inertia coord derivative */
    double _TBID[3][3]; /* *io (--)    Transformation Matrix of body coord wrt inertia coord derivative */

    arma::vec TBI_Q;    /* *io (--)    Transformation Matrix of body coord wrt inertia coord (Quaternion) */
    double _TBI_Q[4];   /* *io (--)    Transformation Matrix of body coord wrt inertia coord (Quaternion) */

    arma::vec TBID_Q;   /* *io (--)    Transformation Matrix of body coord wrt inertia coord derivative (Quaternion) */
    double _TBID_Q[4];  /* *io (--)    Transformation Matrix of body coord wrt inertia coord derivative (Quaternion) */

    arma::mat TBD;      /* *io (--)    Transformation Matrix of body coord wrt geodetic coord */
    double _TBD[3][3];  /* *io (--)    Transformation Matrix of body coord wrt geodetic coord */

    arma::vec TBDQ;     /* *o  (--)    TBDQ */
    double _TBDQ[4];    /* *o  (--)    TBDQ */

    arma::vec VBAB;     /* *o  (m/s)    Air speed in body frame */
    double _VBAB[3];    /* *o  (m/s)    Air speed in body frame */

    arma::mat WEII_skew;     /* *o  (r/s)    Earth's angular velocity (skew-sym) */
    double _WEII_skew[3][3];    /* **  (r/s)    Earth's angular velocity (skew-sym) */

    arma::vec SBIIP;      /* *o  (m)      Vehicle position in inertia coord */
    double _SBIIP[3];       /* *o  (m)      Vehicle position in inertia coord */

    arma::vec VBIIP;      /* *o  (m/s)    Vehicle inertia velocity */
    double _VBIIP[3];       /* *o  (m/s)    Vehicle inertia velocity */

    arma::vec SBII;      /* *o  (m)      Vehicle position in inertia coord */
    double _SBII[3];       /* *o  (m)      Vehicle position in inertia coord */

    arma::vec VBII;      /* *o  (m/s)    Vehicle inertia velocity */
    double _VBII[3];       /* *o  (m/s)    Vehicle inertia velocity */

    arma::vec ABII;      /* *o  (m/s2)   Vehicle inertia acceleration */
    double _ABII[3];      /* *o  (m/s2)   Vehicle inertia acceleration */

    arma::vec ABIB;        /* *o  (m/s2)   Vehicle inertia acceleration on body coordinate */
    double _ABIB[3];       /* *o  (m/s2)   Vehicle inertia acceleration on body coordinate */

    arma::vec SBEE;        /* *o  (m)     Vehicle position in earth coord  */
    double _SBEE[3];           /* *o  (m)     Vehicle position in earth coord  */

    arma::vec VBEE;         /* *o  (m/s)     Vehicle speed in earth coord  */
    double _VBEE[3];           /* *o  (m/s)     Vehicle speed in earth coord  */

    arma::vec ABEE;         /* *o  (m/s2)   Vehicle acceleration in ECEF */
    double _ABEE[3];        /* *o  (m/s2)   Vehicle acceleration in ECEF */

    arma::vec SBEE_old;        /* *o  (m)     Vehicle position in earth coord  */
    double _SBEE_old[3];           /* *o  (m)     Vehicle position in earth coord  */

    arma::vec VBEE_old;         /* *o  (m/s)     Vehicle speed in earth coord  */
    double _VBEE_old[3];           /* *o  (m/s)     Vehicle speed in earth coord  */

    arma::vec ABEE_old;         /* *o  (m/s2)   Vehicle acceleration in ECEF */
    double _ABEE_old[3];        /* *o  (m/s2)   Vehicle acceleration in ECEF */

    arma::vec JBII;         /* *o (m/s3)    Vehicle Jerk in ECI*/
    double _JBII[3];        /* *o (m/s3)    Vehicle Jerk in ECI*/

    arma::vec JBEE;         /* *o (m/s3)    Vehicle Jerk in ECEF */
    double _JBEE[3];        /* *o (m/s3)    Vehicle Jerk in ECEF */

    arma::mat TDI;      /* **  (--)     Transformation Matrix of geodetic wrt inertial  coordinates */
    double _TDI[3][3];     /* **  (--)     Transformation Matrix of geodetic wrt inertial  coordinates */

    arma::mat TGI;      /* **  (--)     Transformation Matrix geocentric wrt inertia coord */
    double _TGI[3][3];     /* **  (--)     Transformation Matrix geocentric wrt inertia coord */

    arma::vec VBED;       /* *o (m/s)   NED velocity */
    double _VBED[3];      /* *o (m/s)   NED velocity */

    arma::vec FSPB;       /* *o  (m/s2)   Specific force in body coord */
    double _FSPB[3];       /* *o  (m/s2)   Specific force in body coord */

    arma::vec CONING;       /* *o (r/s)    Coning angular rate */
    double _CONING[3];      /* *o (r/s)    Coning angular rate */

    arma::vec NEXT_ACC;     /* *o (m/s2)   New Inertial acceleration */
    double _NEXT_ACC[3];    /* *o (m/s2)   New Inertial acceleration */

    arma::mat TDE;
    double _TDE[3][3];

    arma::vec VBII_old;
    double _VBII_old[3];

    arma::vec WEII; /* ** */
    double _WEII[3]; /* ** */

    arma::vec WBII;   /* *io (r/s)        Vehicle's inertia angular velocity in inertia coord */
    double _WBII[3];  /* *io (r/s)        Vehicle's inertia angular velocity in inertia coord */

    arma::vec WBEB;   /* *io (r/s)        Angular velocity of vehicle wrt earth in body coord */
    double _WBEB[3];  /* *io (r/s)        Angular velocity of vehicle wrt earth in body coord */

    arma::vec WBIB;   /* *io (r/s)        Augular velocity of vehicle wrt inertia in body coord */
    double _WBIB[3];  /* *io (r/s)        Augular velocity of vehicle wrt inertia in body coord */

    arma::vec WBIBD;  /* *io (r/s2)       Angular velocity of vehicle wrt inertia in body coord - derivative */
    double _WBIBD[3]; /* *io (r/s2)       Angular velocity of vehicle wrt inertia in body coord - derivative */

    arma::vec TVD;         /* **  (--)     Transformation Matrix of geographic velocity wrt geodetic coord */
    double _TVD[3][3];     /* **  (--)    Transformation Matrix of geographic velocity wrt geodetic coord */

    arma::vec SBEE_test;        /* *o  (m)     Vehicle position in earth coord  */
    double _SBEE_test[3];           /* *o  (m)     Vehicle position in earth coord  */

    arma::vec VBEE_test;         /* *o  (m/s)     Vehicle speed in earth coord  */
    double _VBEE_test[3];           /* *o  (m/s)     Vehicle speed in earth coord  */

    arma::vec ABEE_test;         /* *o  (m/s2)   Vehicle acceleration in ECEF */
    double _ABEE_test[3];        /* *o  (m/s2)   Vehicle acceleration in ECEF */

    arma::mat TLI;
    double _TLI[3][3];

    arma::vec LT_euler;
    double _LT_euler[3];

    arma::vec TBLQ;
    double _TBLQ[4];

    double dang_slosh_theta;
    double ang_slosh_theta;
    double dang_slosh_psi;
    double ang_slosh_psi;

    /* Generating Outputs */
    double ortho_error; /* *io (--)    Direction cosine matrix orthogonality error*/
    double alphax;      /* *io (d)     Angle of attack */
    double betax;       /* *io (d)     Sideslip angle */
    double alppx;       /* *io (d)     Total angle of attack */
    double phipx;       /* *io (d)     Aerodynamic roll angle*/
    double alphaix;     /* *io (d)     Angle of attack, inertia velocity*/
    double betaix;      /* *io (d)     Sideslip angle, inertia velocity*/
    double psibdx;      /* *o (d)     Yaw angle of Vehicle wrt geodetic coord - deg */
    double thtbdx;      /* *o (d)     Pitch angle of Vehicle wrt geodetic coord - deg */
    double phibdx;      /* *o (d)     Roll angle of Vehicle wrt geodetic coord - deg */
    double psibd;      /* *o (r)     Yaw angle of Vehicle wrt geodetic coord - rad */
    double thtbd;      /* *o (r)     Pitch angle of Vehicle wrt geodetic coord - rad */
    double phibd;      /* *o (r)     Roll angle of Vehicle wrt geodetic coord - rad */
    double alt;           /* *o  (m)      Vehicle altitude */
    double lonx;          /* *o  (d)      Vehicle longitude */
    double latx;          /* *o  (d)      Vehicle latitude */
    double _aero_loss;     /* *o  (m/s)    Velocity loss caused by aerodynamic drag */
    double gravity_loss;  /* *o  (m/s)    Velocity loss caused by gravity */
    double t;               /* *o (s)       timer */
    double con_ang;         /* *o (r)      Coning angle */
    double con_w;           /* *o  (--)     Coning freqency */
    double _grndtrck;      /* *o  (m)     [DIAG] Vehicle ground track on earth */
    double _gndtrkmx;      /* *o  (km)    [DIAG] Ground track - km */
    double _gndtrnmx;      /* **  (nm)    [DIAG] Ground track - nm */
    double _ayx;           /* *o  (m/s2)  [DIAG] Achieved side acceleration */
    double _anx;           /* *o  (m/s2)  [DIAG] Achieved normal acceleration */
    double _dbi;           /* *o  (m)     [DIAG] Vehicle distance from center of earth */
    double _dvbi;          /* *o  (m/s)   [DIAG] Vehicle inertia speed */
    double _dvbe;          /* *o  (m/s)   [DIAG] Vehicle geographic speed */
    double _thtvdx;        /* *o  (d)     [DIAG] Vehicle's flight path angle */
    double _psivdx;        /* *o  (d)     [DIAG] Vehicle's heading angle */
    unsigned int liftoff;        /* *i  (--)     To check wether the rocket liftoff or not: liftoff = 1, not liftoff = 0 */
    double ppx;     /* *io (d/s)        Body roll angular velocity wrt earth in body axes */
    double qqx;     /* *io (d/s)        Body pitch angular velocity wrt earth in body axes */
    double rrx;     /* *io (d/s)        Body yaw angular velocity wrt earth in body axes */
    double control_loss;

    /* Orbital Logging */
    double _inclination;   /* *o  (deg)   [DIAG] Orbital inclination is the minimun angle between reference plane and the orbital plane or direction of an object in orbit around another object */
    double _eccentricity;  /* *o  (--)    [DIAG] Determines the amount by which its orbit around another body deviates from a perfect circle */
    double _semi_major;    /* *o  (m)     [DIAG] the major axis of an ellipse is its longest diameter */
    double _ha;            /* *o  (m)     [DIAG] Orbital Apogee */
    double _hp;            /* *o  (m)     [DIAG] Orbital Perigee */
    double _lon_anodex;    /* *o  (deg)   [DIAG] The longitude of the ascending node (☊ or Ω) is one of the orbital elements used to specify the orbit of an object in space. It is the angle from a reference direction, called the origin of longitude, to the direction of the ascending node, measured in a reference plane */
    double _arg_perix;     /* *o  (deg)   [DIAG] The argument of periapsis (also called argument of perifocus or argument of pericenter), symbolized as ω, is one of the orbital elements of an orbiting body. Parametrically, ω is the angle from the body's ascending node to its periapsis, measured in the direction of motion */
    double _true_anomx;    /* *o  (deg)   [DIAG] In celestial mechanics, true anomaly is an angular parameter that defines the position of a body moving along a Keplerian orbit. It is the angle between the direction of periapsis and the current position of the body, as seen from the main focus of the ellipse (the point around which the object orbits) */
    double _ref_alt;       /* *o  (m)     [DIAG] */
    double reference_point;  /* *o (m)    Multibody dynamics reference point */
    double Roll;
    double Pitch;
    double Yaw;

    unsigned int Interpolation_Extrapolation_flag;
};
#endif
