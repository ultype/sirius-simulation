#ifndef __TVC_HH__
#define __TVC_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the TVC Module On Board)
LIBRARY DEPENDENCY:
      ((../src/rocket/Tvc.cpp))
*******************************************************************************/

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
        TVC(Environment &env, Kinematics &kins, Control &con, Propulsion &plp);
        TVC(const TVC& other);

        TVC& operator=(const TVC& other);

        void initialize();

        void actuate(double int_step);
        void tvc_scnd(double &eta, double &zet, double etac, double zetc, double int_step);

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

        double get_parm();
        Matrix get_FPB();
        Matrix get_FMPB();

    private:
        /* Internal Getter */

        /* Internal Initializers */
        void default_data();

        /* Internal Propagator / Calculators */

        /* Internal Calculators */

        /* Routing references */
        Environment * environment;
        Kinematics  * kinematics;
        Control     * control;
        Propulsion  * propulsion;

        /* State */
        enum TVC_TYPE mtvc;            /* *o  (--)      see TVC_TYPE */

        /* Constants */
        double gtvc;        /* *io  (--)    TVC nozzle deflection gain n*/

        double tvclimx;     /* *io  (d)     Nozzle deflection limiter */
        double dtvclimx;    /* *io  (d/s)   Nozzle deflection rate limiter */
        double wntvc;       /* *io  (r/s)   Natural frequency of TVC */
        double zettvc;      /* *io  (--)    Damping of TVC */
        double factgtvc;    /* *io  (--)    Factor for TVC gain */

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
        double fpb[3];      /* *o  (N)     Thrust force in body axes n*/
        double fmpb[3];     /* *o  (N*m)    Thrust moment in body axes */

        double etax;        /* *io  (d)     Nozzle pitch deflection */
        double zetx;        /* *io  (d)     Nozzle yaw deflection */

        /* Non-propagating Diagnostic Variables */
        /* These can be deleted, but keep to remain trackable in trick simulator */

        double etacx;       /* *io  (d)     Commanded nozzle pitch deflection */
        double zetcx;       /* *io  (d)     Commanded nozzle yaw deflection */

};

#endif  // __TVC_HH__
