#ifndef __INS_HH__
#define __INS_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the INS Module On Board)
LIBRARY DEPENDENCY:
      ((../src/rocket/Ins.cpp))
PROGRAMMERS:
      (((Chung-Fan Yang) () () () ))
*******************************************************************************/

#include "Newton.hh"
#include "Euler.hh"
#include "Environment.hh"
#include "Kinematics.hh"
#include "GPS_receiver.hh"

#include "sensor/gyro/gyro.hh"
#include "sensor/accel/accelerometer.hh"

class GPS_Receiver;
class Newton;
class Kinematics;
class _Euler_;
class Environment;

class INS {
    TRICK_INTERFACE(INS);
    public:
        INS() {}

        void default_data();
        void initialize(Newton *ntn, _Euler_ *elr, Environment *env, Kinematics *kins, GPS_Receiver *gps);

        void ins_grav();
        void update(double int_step);

        Newton *newton;
        _Euler_ *euler;
        Environment *environment;
        Kinematics *kinematics;
        GPS_Receiver *gpsr;

        void set_gyro(sensor::Gyro &gyro);
        void set_accelerometer(sensor::Accelerometer &accel);

        sensor::Gyro& get_gyro();
        sensor::Accelerometer& get_accelerometer();

        void set_ideal();
        void set_non_ideal();

        /* Input File */
        enum INS_TYPE {
            IDEAL_INS = 0,
            NON_IDEAL_INS = 1
        };

        double get_dvbec();
        double get_qqcx();
        double get_rrcx();
        double get_ppcx();
        double get_alphacx();
        double get_betacx();
        double get_phibdcx();
        double get_thtbdcx();
        double get_psibdcx();

        Matrix get_SBIIC();
        Matrix get_VBIIC();
        Matrix get_WBICI();
        Matrix get_EGRAVI();

        void set_SBIIC(double, double, double);
        void set_VBIIC(double, double, double);
        void set_WBICI(double, double, double);
        void set_EGRAVI(double, double, double);

        Matrix get_TBIC();

    private:
        enum INS_TYPE ins_mode;  /* *o  (--)    INS mode. see INS_TYPE */

        double dvbec;       /* *io  (m/s)   Computed body speed wrt earth */
        double qqcx;        /* *io  (d/s)   INS computed pitch rate */
        double rrcx;        /* *io  (d/s)   INS computed yaw rate */
        double fspcb[3];    /* *io  (N/kg)  Computed specific force on body */
        double sbiic[3];    /* *io  (m)     Computed pos of body wrt earth reference point*/
        double vbiic[3];    /* *io  (m/s)   Computed body vel in earth coor */
        double wbici[3];    /* *io  (r/s)   Computed inertial body rate in inert coordinate */
        double tbic[3][3];  /* *io  (--)    Comp T.M. of body wrt earth coordinate */
        double ppcx;        /* *io  (d/s)   INS computed roll rate */
        double alphacx;     /* *io  (d)     INS computed angle of attack */
        double betacx;      /* *io  (d)     INS computed sideslip angle */
        double phibdcx;     /* *io  (d)     INS computed geodetic Euler roll angle */
        double thtbdcx;     /* *io  (d)     INS computed geodetic Euler pitch angle */
        double psibdcx;     /* *io  (d)     INS computed geodetic Euler yaw angle */

        /* Sensors */
        sensor::Gyro * gyro;
        sensor::Accelerometer * accel;

        /* grav */
        double egravi[3];   /* *i   (--)    error by gravity */

        double frax_algnmnt;/* *io  (--)    Fractn to mod initial INS err state: XXO=XXO(1+frax) */
        double wbicb[3];    /* *io  (r/s)   Computed inertial body rate in body coordinate */
        double loncx;       /* *io  (d)     INS derived longitude */
        double latcx;       /* *io  (d)     INS derived latitude */
        double altc;        /* *io  (m)     INS derived altitude */
        double vbecd[3];    /* *io  (m/s)   Geodetic velocity */
        double tdci[3][3];  /* *io  (--)    Comp T.M. of geodetic wrt inertial */
        double thtvdcx;     /* *io  (d)     INS computed vertical flight path angle */
        double psivdcx;     /* *io  (d)     INS computed heading angle */
        double dbic;        /* *io  (m)     INS computed vehicle distance from Earth center */
        double alppcx;      /* *io  (d)     INS computed total angle of attack */
        double phipcx;      /* *io  (d)     INS computed aero roll angle */
        double evbid[3];    /* *io  (m/s)   INS vel error derivative */
        double evbi[3];     /* *io  (m/s)   INS vel error */
        double esbid[3];    /* *io  (m)     INS pos error derivative */
        double esbi[3];     /* *io  (m)     INS pos error */
        double ins_pos_err; /* *io  (m)     INS absolute postion error */
        double ins_vel_err; /* *io  (m/s)   INS absolute velocity error */
};

#endif  // __INS_HH__
