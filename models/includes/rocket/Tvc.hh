#ifndef __TVC_HH__
#define __TVC_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the TVC Module On Board)
LIBRARY DEPENDENCY:
      ((../src/rocket/Tvc.cpp))
*******************************************************************************/
#include <tuple>

#include "aux/aux.hh"

#include "Newton.hh"
#include "Euler.hh"
#include "Environment.hh"
#include "Kinematics.hh"
#include "GPS_receiver.hh"
#include "Control.hh"

class Propulsion;
class Kinematics;
class Control;
class Environment;

class TVC {
    TRICK_INTERFACE(TVC);

    public:
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version){
            ar & environment;
            ar & kinematics;
            ar & control;
            ar & propulsion;

            ar & mtvc;

            /* Constants */
            ar & gtvc;

            ar & tvclimx;
            ar & dtvclimx;
            ar & wntvc;
            ar & zettvc;
            ar & factgtvc;

            /* Propagative Stats */
            ar & etas;
            ar & etasd;

            ar & zeta;
            ar & zetad;

            ar & detas;
            ar & detasd;

            ar & dzeta;
            ar & dzetad;

            /* Generating Outputs */
            ar & parm;

            ar & _FPB;

            ar & _FMPB;

            ar & etax;
            ar & zetx;

            /* Non-propagating Diagnostic Variables */
            /* These can be deleted, but keep to remain trackable in trick simulator */
            ar & etacx;
            ar & zetcx;
        }

        TVC(Environment &env, Kinematics &kins, Control &con, Propulsion &plp);
        TVC(const TVC& other);

        TVC& operator=(const TVC& other);

        void initialize();

        void actuate(double int_step);

        enum TVC_TYPE {
            NO_TVC = 0,
            NO_DYNAMIC_TVC,
            SECON_ORDER_TVC,        // TVC Second order dynamics with rate limiting
            ONLINE_SECOND_ORDER_TVC // same as 2nd order but with on-line TVC gain
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

        double get_parm();

        arma::vec3 get_FPB();
        arma::vec3 get_FMPB();

    private:
        /* Internal Getter */

        /* Internal Initializers */
        void default_data();

        /* Internal Propagator / Calculators */
        // returning: eta, zet,
        std::tuple<double, double> tvc_scnd(double etac, double zetc, double int_step);

        arma::vec3 calculate_FPB(double eta, double zet, double thrust);
        arma::vec3 calculate_FMPB(double xcg);

        /* Internal Calculators */

        /* Routing references */
        Environment * environment;
        Kinematics  * kinematics;
        Control     * control;
        Propulsion  * propulsion;

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

};

#endif  // __TVC_HH__
