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

        void ins_accl();
        void ins_gyro(double int_step);
        void ins_grav();
        void update(double int_step);

        Newton *newton;
        _Euler_ *euler;
        Environment *environment;
        Kinematics *kinematics;
        GPS_Receiver *gpsr;

        /* Input File */
        int mins;           /* *io  (--)    INS mode. =0:ideal INS; =1:with INS error */

        double get_dvbec();
        double get_qqcx();
        double get_rrcx();
        double get_ppcx();
        double get_alphacx();
        double get_betacx();
        double get_phibdcx();
        double get_thtbdcx();
        double get_psibdcx();

        Matrix get_FSPCB();
        Matrix get_SBIIC();
        Matrix get_VBIIC();
        Matrix get_WBICI();
        Matrix get_EFSPB();
        Matrix get_EWALKA();
        Matrix get_EMISA();
        Matrix get_ESCALA();
        Matrix get_EBIASA();
        Matrix get_EUG();
        Matrix get_EWG();
        Matrix get_EWBIB();
        Matrix get_EWALKG();
        Matrix get_EUNBG();
        Matrix get_EMISG();
        Matrix get_ESCALG();
        Matrix get_EBIASG();
        Matrix get_EGRAVI();

        void set_FSPCB(double, double, double);
        void set_SBIIC(double, double, double);
        void set_VBIIC(double, double, double);
        void set_WBICI(double, double, double);
        void set_EFSPB(double, double, double);
        void set_EWALKA(double, double, double);
        void set_EMISA(double, double, double);
        void set_ESCALA(double, double, double);
        void set_EBIASA(double, double, double);
        void set_EUG(double, double, double);
        void set_EWG(double, double, double);
        void set_EWBIB(double, double, double);
        void set_EWALKG(double, double, double);
        void set_EUNBG(double, double, double);
        void set_EMISG(double, double, double);
        void set_ESCALG(double, double, double);
        void set_EBIASG(double, double, double);
        void set_EGRAVI(double, double, double);

        Matrix get_TBIC();

    private:
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

        /* Accelmeter */
        double efspb[3];    /* *i   (N/kg)  Error in specific force on body in body coordinate */
        double ewalka[3];   /* *i   (m/s2)  Acceleration random noise */
        double emisa[3];    /* *i   (r)     Acceleration misalignment */
        double escala[3];   /* *i   (--)    Acceleration scale factor */
        double ebiasa[3];   /* *i   (m/s2)  Acceleration bias */

        /* gyro */
        double eug[3];      /* *i   (r/s)   Gyro spin axis accel sensitivity */
        double ewg[3];      /* *i   (r/s)   Gyro random walk errors */
        double ewbib[3];    /* *i   (r/s)   Error in angular vel of body wrt earth */
        double ewalkg[3];   /* *i   (r/s0.5) Random walk */
        double eunbg[3];    /* *i   (r)     Gyro cluster misalignment */
        double emisg[3];    /* *i   (r)     Gyro misalignmt */
        double escalg[3];   /* *i   (--)    Gyro scale factor */
        double ebiasg[3];   /* *i   (r/s)   Gyro bias */

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
        double ricid[3];    /* *io  (r)     INS tilt error derivative */
        double rici[3];     /* *io  (r)     INS tilt error */
        double evbid[3];    /* *io  (m/s)   INS vel error derivative */
        double evbi[3];     /* *io  (m/s)   INS vel error */
        double esbid[3];    /* *io  (m)     INS pos error derivative */
        double esbi[3];     /* *io  (m)     INS pos error */
        double ins_pos_err; /* *io  (m)     INS absolute postion error */
        double ins_vel_err; /* *io  (m/s)   INS absolute velocity error */
        double ins_tilt_err;/* *io  (r)     INS absolute tilt error */
};

#endif  // __INS_HH__