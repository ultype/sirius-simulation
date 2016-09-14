#ifndef __GPS_R_HH__
#define __GPS_R_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the GPS Receiver On Board)
LIBRARY DEPENDENCY:
      ((../src/GPS_receiver.cpp) (../src/utility_functions.cpp))
PROGRAMMERS:
      (((Chung-Fan Yang) () () () ))
*******************************************************************************/

#include "Newton.hh"
#include "Euler.hh"
#include "Ins.hh"
#include "GPS_satellites.hh"

class INS;

class GPS_Receiver {
    public:
    GPS_Receiver() {}

    void default_data();
    void initialize(Newton*, _Euler_*, GPS_Satellites*, INS*, double);

    void get_quadriga();

    void filter_extrapolation(double);
    void measure();

    Newton *newton;
    _Euler_ *euler;
    GPS_Satellites *gps_sats;
    INS *ins;

    /* Internal variables */
    bool gps_acq;               /* ** (--)      GPS Signal Acquired? */
    double gps_epoch;           /* ** (s)       GPS update epoch time since launch */
    double time_gps;            /* ** (s)       GPS time passed */
    Matrix FF = Matrix(8, 8);   /* ** (--)      Constant*/
    Matrix PHI = Matrix(8, 8);  /* ** (--)      Constant*/

    //XXX: use C array, for check-pointing and variable server
    Matrix PP1 = Matrix(3, 3);  /* ** (--)      Covariance Matrix 1st row */
    Matrix PP2 = Matrix(3, 3);  /* ** (--)      Covariance Matrix 2st row */
    Matrix PP3 = Matrix(3, 3);  /* ** (--)      Covariance Matrix 3st row */
    Matrix PP4 = Matrix(3, 3);  /* ** (--)      Covariance Matrix 4st row */
    Matrix PP5 = Matrix(3, 3);  /* ** (--)      Covariance Matrix 5st row */
    Matrix PP6 = Matrix(3, 3);  /* ** (--)      Covariance Matrix 6st row */
    Matrix PP7 = Matrix(3, 3);  /* ** (--)      Covariance Matrix 7st row */
    Matrix PP8 = Matrix(3, 3);  /* ** (--)      Covariance Matrix 8st row */

    double slotsum;         /* ** (--)      Sum of stored slot numbers of quadriga */

    /* XXX: GPS Executing Parameter */
    /* These will be affected by S_define */

    /* GPS Device parameters */
    double del_rearth;      /* *i (m)       GPS Receiver LOS Minimum distance */
    double gps_acqtime;     /* *i (s)       Time to Acquire GPS Signal */
    double gps_step;        /* *i (s)       GPS Update Interval */

    double ucfreq_noise;    /* *i (m/s)     User clock frequency error XXX: MARKOV */
    double ucbias_error;    /* *io (m)      User clock bias error XXX: GAUSS */

    double PR_BIAS[4];      /* *i (m)       Pseudo-range bias GAUSS */
    double PR_NOISE[4];     /* *i (m)       Pseudo-range bias MARKOV */
    double DR_NOISE[4];     /* *i (m/s)     Delta-range noise MARKOV */

    /* GPS EKF Parameters */
    double uctime_cor;      /* *i (s)       User clock correlation time constant */
    double ppos;            /* *i (m)       Init 1sig pos values of state cov matrix */
    double pvel;            /* *i (m/s)     Init 1sig vel values of state cov matrix */
    double pclockb;         /* *i (m)       Init 1sig clock bias error of state cov matrix */
    double pclockf;         /* *i (m/s)     Init 1sig clock freq error of state cov matrix */
    double qpos;            /* *i (m)       1sig pos values of process cov matrix */
    double qvel;            /* *i (m/s)     1sig vel values of process cov matrix */
    double qclockb;         /* *i (m)       1sig clock bias error of process cov matrix */
    double qclockf;         /* *i (m/s)     1sig clock freq error of process cov matrix */
    double rpos;            /* *i (m)       1sig pos value of meas cov matrix */
    double rvel;            /* *i (m/s)     1sig vel value of meas cov matrix */
    double factp;           /* *i (--)      Factor to modifiy initial P-matrix P(1+factp) */
    double factq;           /* *i (--)      Factor to modifiy the Q-matrix Q(1+factq) */
    double factr;           /* *i (--)      Factor to modifiy the R-matrix R(1+factr) */

    /* GPS Outputs */
    int gps_update;       /* *o (--)       GPS update? > 0 updated */
    double gdop;            /* *o (m)        Geometric dillution of precision of quadriga */
    double slot[4];         /* *o (--)       SV slot#  of quadriga */
    double ssii_quad[16];   /* *o (m)        Best quadriga inertial coordinates and their slot# */
    double vsii_quad[12];   /* *o (m/s)      Best quadriga inertial velocities */

    double ucfreq_error;    /* *o (m)        User clock frequency error */
    double ucfreqm;         /* *o (m/s)      User clock frequency state */

    double std_pos;         /* *o (m)        std deviation of position from p matrix */
    double std_vel;         /* *o (m)        std deviation of velocity from p matrix */
    double std_ucbias;      /* *o (m)        std deviation of user clock bias from p matrix */

    double lat1;            /* *o (d)        Quadriga 1st lat */
    double lat2;            /* *o (d)        Quadriga 2nd lat */
    double lat3;            /* *o (d)        Quadriga 3rd lat */
    double lat4;            /* *o (d)        Quadriga 4th lat */
    double lon1;            /* *o (d)        Quadriga 1st lon */
    double lon2;            /* *o (d)        Quadriga 2nd lon */
    double lon3;            /* *o (d)        Quadriga 3rd lon */
    double lon4;            /* *o (d)        Quadriga 4th lon */
    double alt1;            /* *o (m)        Quadriga 1st alt */
    double alt2;            /* *o (m)        Quadriga 2nd alt */
    double alt3;            /* *o (m)        Quadriga 3rd alt */
    double alt4;            /* *o (m)        quadriga 4th alt */

    double position_state[3];  /* *o (m)        Position state (inertial coor) */
    double velocity_state[3];  /* *o (m)        Velocity state (inertial coor) */
    double clock_state[3];     /* *o (--)       CLock state */

    double gps_pos_meas;    /* *o (m)        */
    double gps_vel_meas;    /* *o (m)        */
    double state_pos;    /* *o (m)        */
    double state_vel;    /* *o (m)        */
};

#endif  // __GPS_R_HH__
