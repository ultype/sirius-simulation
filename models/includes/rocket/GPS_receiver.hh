#ifndef __GPS_R_HH__
#define __GPS_R_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the GPS Receiver On Board)
LIBRARY DEPENDENCY:
      ((../src/rocket/GPS_receiver.cpp) (../src/aux/utility_functions.cpp))
PROGRAMMERS:
      (((Chung-Fan Yang) () () () ))
*******************************************************************************/

#include "Newton.hh"
#include "Euler.hh"
#include "Ins.hh"
#include "GPS_satellites.hh"

class INS;
class Newton;
class _Euler_;

class GPS_Receiver {
    TRICK_INTERFACE(GPS_Receiver);

    public:
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version){
            /* Input File */
            ar & slot;

            /* GPS Device parameters */
            ar & del_rearth;
            ar & gps_acqtime;
            ar & gps_step;

            ar & ucfreq_noise;
            ar & ucfreq_noise_sigma;
            ar & ucfreq_noise_bcor;
            ar & ucbias_error;

            ar & PR_BIAS;
            ar & PR_NOISE;
            ar & PR_NOISE_sigma;
            ar & PR_NOISE_bcor;
            ar & DR_NOISE;
            ar & DR_NOISE_sigma;
            ar & DR_NOISE_bcor;

            /* GPS EKF Parameters */
            ar & ppos;
            ar & pvel;
            ar & qpos;
            ar & qvel;
            ar & rpos;
            ar & rvel;
            ar & factr;

            /* Internal variables */
            ar & gps_acq;
            ar & gps_epoch;
            ar & time_gps;

            ar & _FF;
            ar & _PHI;
            ar & _PP;

            ar & factq;
            ar & qclockb;
            ar & qclockf;

            ar & slotsum;

            ar & _SXH;
            ar & _VXH;
            ar & _CXH;

            /* GPS Outputs */
            ar & gdop;
            ar & ssii_quad;
            ar & vsii_quad;

            ar & ucfreq_error;
            ar & ucfreqm;

            ar & std_pos;
            ar & std_vel;
            ar & std_ucbias;

            ar & lat1;
            ar & lat2;
            ar & lat3;
            ar & lat4;
            ar & lon1;
            ar & lon2;
            ar & lon3;
            ar & lon4;
            ar & alt1;
            ar & alt2;
            ar & alt3;
            ar & alt4;

            ar & gps_pos_meas;
            ar & gps_vel_meas;
            ar & state_pos;
            ar & state_vel;
        }

        GPS_Receiver(Newton&, _Euler_&, GPS_Satellites&, INS&);
        GPS_Receiver(const GPS_Receiver& other);

        GPS_Receiver& operator=(const GPS_Receiver& other);

        /* factp    (--)      Factor to modifiy initial P-matrix P(1+factp) */
        /* pclockb  (m)       Init 1sig clock bias error of state cov matrix */
        /* pclockf  (m/s)     Init 1sig clock freq error of state cov matrix */
        void setup_state_covariance_matrix(double factp, double pclockb, double pclockf);

        /* factq    (--)      Factor to modifiy the Q-matrix Q(1+factq) */
        /* qclockb  (m)       1sig clock bias error of process cov matrix */
        /* qclockf  (m/s)     1sig clock freq error of process cov matrix */
        void setup_error_covariance_matrix(double factq, double qclockb, double qclockf);

        /* uctime_cor (s)       User clock correlation time constant */
        void setup_fundamental_dynamic_matrix(double uctime_cor);

        void initialize(double int_step);
        void update_markov(double);

        void get_quadriga();

        void filter_extrapolation(double);
        void measure();

        int gps_update;       /* *o (--)       GPS update? > 0 updated */

        arma::vec3 get_SXH();
        arma::vec3 get_VXH();
        arma::vec3 get_CXH();

        /* Input File */
        double slot[4];         /* *o (--)       SV slot#  of quadriga */

        /* GPS Device parameters */
        double del_rearth;      /* *i (m)       GPS Receiver LOS Minimum distance */
        double gps_acqtime;     /* *i (s)       Time to Acquire GPS Signal */
        double gps_step;        /* *i (s)       GPS Update Interval */

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

        /* GPS EKF Parameters */
        double ppos;            /* *i (m)       Init 1sig pos values of state cov matrix */
        double pvel;            /* *i (m/s)     Init 1sig vel values of state cov matrix */
        double qpos;            /* *i (m)       1sig pos values of process cov matrix */
        double qvel;            /* *i (m/s)     1sig vel values of process cov matrix */
        double rpos;            /* *i (m)       1sig pos value of meas cov matrix */
        double rvel;            /* *i (m/s)     1sig vel value of meas cov matrix */
        double factr;           /* *i (--)      Factor to modifiy the R-matrix R(1+factr) */
    private:
        void default_data();

        Newton         * newton;
        _Euler_        * euler;
        GPS_Satellites * gps_sats;
        INS            * ins;

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

        double slotsum;         /* ** (--)      Sum of stored slot numbers of quadriga */

        arma::vec SXH;       /* *o (m)        Position state (inertial coor) */
        double   _SXH[3];    /* *o (m)        Position state (inertial coor) */

        arma::vec VXH;      /* *o (m)        Velocity state (inertial coor) */
        double   _VXH[3];   /* *o (m)        Velocity state (inertial coor) */

        arma::vec CXH;      /* *o (--)       CLock state */
        double   _CXH[3];   /* *o (--)       CLock state */

        /* GPS Outputs */
        double gdop;            /* *o (m)        Geometric dillution of precision of quadriga */
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

        double gps_pos_meas;    /* *o (m)        */
        double gps_vel_meas;    /* *o (m)        */
        double state_pos;    /* *o (m)        */
        double state_vel;    /* *o (m)        */
};

#endif  // __GPS_R_HH__
