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

#include "Newton.hh"
#include "Euler.hh"

class Newton;
class Environment;
class _Euler_;

class Kinematics{
    TRICK_INTERFACE(Kinematics);

    public:
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version){
            ar & newton;
            ar & environment;
            ar & euler;

            ar & _TBD;
            ar & _TBI;
            ar & _TBID;
            ar & alphax;
            ar & betax;
            ar & alppx;
            ar & phipx;
            ar & psibdx;
            ar & thtbdx;
            ar & phibdx;
            ar & ortho_error;
            ar & alphaix;
            ar & betaix;
        }

        Kinematics(Newton &newt, Environment &env, _Euler_ &eul);
        Kinematics(const Kinematics& other);

        Kinematics& operator=(const Kinematics& other);

        void initialize();

        void propagate(double int_step);
        void update_diagnostic_attributes(double int_step);

        arma::mat get_TBD();
        arma::mat get_TBI();

        double get_alppx();
        double get_phipx();
        double get_alphax();
        double get_betax();
        double get_psibdx();
        double get_thtbdx();
        double get_phibdx();

        void load_angle(double yaw, double roll, double pitch);

    private:
        void default_data();

        /* Internal Getter */
        double get_thtbdx_in(double &cthtbd);

        /* Internal Initializers */

        /* Internal Propagator / Calculators */
        void propagate_TBI(double int_step, arma::vec3 WBIB);
        void propagate_TBI_Q(double int_step, arma::vec3 WBIB);

        arma::mat calculate_TBD(double lonx, double latx, double alt);

        double calculate_alphaix(arma::vec3 VBIB);
        double calculate_betaix(arma::vec3 VBIB);
        double calculate_alppx(arma::vec3 VBAB, double dvba);
        double calculate_phipx(arma::vec3 VBAB);
        double calculate_alphax(arma::vec3 VBAB);
        double calculate_betax(arma::vec3 VBAB, double dvba);

        /* Internal Calculators */

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

        arma::vec TBI_Q;    /* *io (--)    Transformation Matrix of body coord wrt inertia coord (Quaternion) */
        double _TBI_Q[4];   /* *io (--)    Transformation Matrix of body coord wrt inertia coord (Quaternion) */
        
        arma::vec TBID_Q;   /* *io (--)    Transformation Matrix of body coord wrt inertia coord derivative (Quaternion) */
        double _TBID_Q[4];  /* *io (--)    Transformation Matrix of body coord wrt inertia coord derivative (Quaternion) */

        /* Generating Outputs */
        double ortho_error; /* *io (--)    Direction cosine matrix orthogonality error*/

        arma::mat TBD;      /* *io (--)    Transformation Matrix of body coord wrt geodetic coord */
        double _TBD[3][3];  /* *io (--)    Transformation Matrix of body coord wrt geodetic coord */
        arma::vec TBDQ;     /* *o  (--)     TBDQ */
        double _TBDQ[4];    /* *o  (--)     TBDQ */

        double alphax;      /* *io (d)     Angle of attack */
        double betax;       /* *io (d)     Sideslip angle */
        double alppx;       /* *io (d)     Total angle of attack */
        double phipx;       /* *io (d)     Aerodynamic roll angle*/

        double alphaix;     /* *io (d)     Angle of attack, inertia velocity*/
        double betaix;      /* *io (d)     Sideslip angle, inertia velocity*/

        /* Non-propagating Diagnostic Variables */
        /* These can be deleted, but keep to remain trackable in trick simulator */
        double psibdx;      /* *o (d)     Yaw angle of Vehicle wrt geodetic coord - deg */
        double thtbdx;      /* *o (d)     Pitch angle of Vehicle wrt geodetic coord - deg */
        double phibdx;      /* *o (d)     Roll angle of Vehicle wrt geodetic coord - deg */
        double psibd;      /* *o (r)     Yaw angle of Vehicle wrt geodetic coord - rad */
        double thtbd;      /* *o (r)     Pitch angle of Vehicle wrt geodetic coord - rad */
        double phibd;      /* *o (r)     Roll angle of Vehicle wrt geodetic coord - rad */

};


#endif  // __kinematics_HH__
