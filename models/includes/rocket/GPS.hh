#ifndef __GPS_HH
#define __GPS_HH
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the GPS compute unit On Board)
LIBRARY DEPENDENCY:
      ((../src/rocket/GPS.cpp) (../src/aux/utility_functions.cpp))
PROGRAMMERS:
      (((Lai Chun Hsu) () () () ))
*******************************************************************************/
#include "Ins.hh"

class GPS{

	TRICK_INTERFACE(GPS);
    public:
    	GPS(time_management &time_ma, GPS_constellation &gps_cons);
    	GPS(const GPS &other);
    	GPS & operator= (const GPS &other);

        void setup_state_covariance_matrix(double factp, double pclockb, double pclockf);
        void setup_error_covariance_matrix(double factq, double qclockb, double qclockf);
        void setup_fundamental_dynamic_matrix(double uctime_cor);

    private:
        time_management * time;
    	GPS_constellation * gps_con;
        void filter_extrapolation(double int_step);
        void measure();

        /* GPS EKF Parameters */
        double ppos;            /* *i (m)       Init 1sig pos values of state cov matrix */
        double pvel;            /* *i (m/s)     Init 1sig vel values of state cov matrix */
        double qpos;            /* *i (m)       1sig pos values of process cov matrix */
        double qvel;            /* *i (m/s)     1sig vel values of process cov matrix */
        double rpos;            /* *i (m)       1sig pos value of meas cov matrix */
        double rvel;            /* *i (m/s)     1sig vel value of meas cov matrix */
        double factr;           /* *i (--)      Factor to modifiy the R-matrix R(1+factr) */

        int gps_update;       /* *o (--)       GPS update? > 0 updated */

        /* Internal variables */
        bool gps_acq;               /* ** (--)      GPS Signal Acquired? */
        double gps_epoch;           /* ** (s)       GPS update epoch time since launch */
        double time_gps;            /* ** (s)       GPS time passed */

        arma::mat FF;             /* *o (--)      Constant*/
        double _FF[8][8];             /* *o (--)      Constant*/

        arma::mat PHI;            /* *o (--)      Constant*/
        double _PHI[8][8];            /* *o (--)      Constant*/

        arma::mat PP;               /* *o (--)      Covariance Matrix */
        double _PP[8][8];           /* *o (--)      Covariance Matrix */

        double factq;               /* *o (--)      Factor to modifiy the Q-matrix Q(1+factq) */
        double qclockb;             /* *o (m)       1sig clock bias error of process cov matrix */
        double qclockf;             /* *o (m/s)     1sig clock freq error of process cov matrix */

        arma::vec SXH;       /* *o (m)        Position state (inertial coor) */
        double   _SXH[3];    /* *o (m)        Position state (inertial coor) */

        arma::vec VXH;      /* *o (m)        Velocity state (inertial coor) */
        double   _VXH[3];   /* *o (m)        Velocity state (inertial coor) */

        arma::vec CXH;      /* *o (--)       CLock state */
        double   _CXH[3];   /* *o (--)       CLock state */

        /* GPS Outputs */
        
        double ucfreq_error;    /* *o (m)        User clock frequency error */
        double ucfreqm;         /* *o (m/s)      User clock frequency state */

        double std_pos;         /* *o (m)        std deviation of position from p matrix */
        double std_vel;         /* *o (m)        std deviation of velocity from p matrix */
        double std_ucbias;      /* *o (m)        std deviation of user clock bias from p matrix */

        double gps_pos_meas;    /* *o (m)        */
        double gps_vel_meas;    /* *o (m)        */
        double state_pos;    /* *o (m)        */
        double state_vel;    /* *o (m)        */

        double ucfreq_noise;    /* *i (m/s)     User clock frequency error */
        double ucfreq_noise_sigma;  /* *i (--)     User clock frequency error sigma of markov variable */
        double ucfreq_noise_bcor;   /* *i (--)     User clock frequency error bcor of markov variable */
        double ucbias_error;    /* *io (m)      User clock bias error */

        double PR_BIAS[4];      /* *i (m)       Pseudo-range bias GAUSS */

        double PR_NOISE[4];     /* *i (m)       Pseudo-range bias */
        double PR_NOISE_sigma[4];    /* *i (--)       Pseudo-range bias sigma */
        double PR_NOISE_bcor[4];     /* *i (--)       Pseudo-range bias bcor */

        double DR_NOISE[4];     /* *i (m/s)     Delta-range noise */
        double DR_NOISE_sigma[4];    /* *i (--)     Delta-range noise sigma */
        double DR_NOISE_bcor[4];     /* *i (--)     Delta-range noise bcor */

};


#endif