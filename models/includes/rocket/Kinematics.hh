#ifndef __kinematics_HH__
#define __kinematics_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the kinematics Module Variables and Algorithm)
LIBRARY DEPENDENCY:
      ((../src/rocket/Kinematics.cpp))
PROGRAMMERS:
      (((Lai Jun Xu) () () () ))
*******************************************************************************/
#include "aux/global_constants.hh"
#include "aux/utility_header.hh"

#include "Newton.hh"
#include "Euler.hh"

class Newton;
class Environment;
class _Euler_;

class Kinematics{
    TRICK_INTERFACE(Kinematics);

    public:
        Kinematics(Newton &newt, Environment &env, _Euler_ &eul);
        Kinematics(const Kinematics& other);

        Kinematics& operator=(const Kinematics& other);

        void initialize();

        void calculate_kinematics(double int_step);

/***********************************Variables describtion******************************/
        //double time;        /* *io (s)     simulation time */
        Matrix get_TBD();
        Matrix get_TBI();
        arma::mat get_TBD_();
        arma::mat get_TBI_();
        double get_alppx();
        double get_phipx();
        double get_alphax();
        double get_betax();
        double get_psibdx();
        double get_thtbdx();
        double get_phibdx();

        void load_angle(double yaw, double roll, double pitch);

        // XXX: can't get from private....
    private:
        void default_data();

        /* Internal Getter */
        double get_thtbdx(double &cthtbd);

        /* Internal Initializers */

        /* Internal Propagator */

        /* Internal Updaters */
        void update_alppx();
        void update_phipx();
        void update_alphax();
        void update_betax();

        /* Routing references */
        Newton      * newton;
        Environment * environment;
        _Euler_     * euler;

        /* Constants */

        /* Propagative Stats */
        arma::mat TBI;      /* *io (--)    Transformation Matrix of body coord wrt inertia coord */
        double _TBI[3][3];  /* *io (--)    Transformation Matrix of body coord wrt inertia coord */

        arma::mat TBID;     /* *io (--)    Transformation Matrix of body coord wrt inertia coord derivative */
        double _TBID[3][3]; /* *io (--)    Transformation Matrix of body coord wrt inertia coord derivative */

        /* Generating Outputs */
        arma::mat TBD;      /* *io (--)    Transformation Matrix of body coord wrt geodetic coord */
        double _TBD[3][3];  /* *io (--)    Transformation Matrix of body coord wrt geodetic coord */

        double alphax;      /* *io (d)     Angle of attack */
        double betax;       /* *io (d)     Sideslip angle */
        double alppx;       /* *io (d)     Total angle of attack */
        double phipx;       /* *io (d)     Aerodynamic roll angle*/

        /* Non-propagating Diagnostic Variables */
        /* These can be deleted, but keep to remain trackable in trick simulator */
        double psibdx;      /* *io (d)     Yaw angle of Vehicle wrt geodetic coord - deg */
        double thtbdx;      /* *io (d)     Pitch angle of Vehicle wrt geodetic coord - deg */
        double phibdx;      /* *io (d)     Roll angle of Vehicle wrt geodetic coord - deg */
        double ortho_error; /* *io (--)    Direction cosine matrix orthogonality error*/
        double alphaix;     /* *io (d)     Angle of attack, inertia velocity*/
        double betaix;      /* *io (d)     Sideslip angle, inertia velocity*/
/**************************************************************************************/
};


#endif  // __kinematics_HH__
