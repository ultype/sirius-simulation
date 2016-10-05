#ifndef __ACTUATOR_HH__
#define __ACTUATOR_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the ACTUATOR Module On Board)
LIBRARY DEPENDENCY:
      ((../src/Actuator.cpp))
PROGRAMMERS:
      (((Chung-Fan Yang) () () () ))
*******************************************************************************/

#include "Newton.hh"
#include "Control.hh"

class Actuator {
    public:
    Actuator() {}

    void default_data();
    void initialize(Control *control);

    void actuate(double int_step);
    Matrix actuator_0th(Matrix ACTCZ, double dlimx, double dlimx_min, int num_fins);
    Matrix actuator_scnd(Matrix ACTCZ, double dlimx, double dlimx_min, int num_fins, double int_step) ;

    Control* control;

    ///////////////////////////////////////////////////////////////////////////////
    // Definition of actuator module-variables
    // Member function of class 'Hyper'
    // Module-variable locations are assigned to hyper[600-649]
    //
    // mact=|morder|mvehicle|
    //
    //    morder = 0 no dynamics, fins are position limited
    //           = 2 second order dynamics, fins are position and rate limited
    //    mvehicle = 1 not used
    //             = 2 Rocket
    //
    ///////////////////////////////////////////////////////////////////////////////
    int     mact;           /* *io  (--)    mact=|morder|mvehicle|, see table */
    int     num_fins;       /* *io  (--)    Number of fins */
    double  dlimx;          /* *io  (d)     Control fin limiter */
    double  dlimx_min;      /* *io  (d)     Minimum fin limiter (optional, usually negative) */
    double  ddlimx;         /* *io  (d/s)   Control fin rate limiter */
    double  wnact;          /* *io  (r/s)   Natural frequency of actuator */
    double  zetact;         /* *io  (--)    Damping of actuator */
    int     mvehicle;       /* *io  (--)    =1:Unused;=2:X51 */
    double  factwnact;      /* *io  (--)    Fact to change bandwidth wnact*(1+factwnact) */
    double  wnact_limit;    /* *io  (r/s)   Maximum bandwidth limit */
    double  delax;          /* *io  (d)     Aileron control deflection */
    double  delex;          /* *io  (d)     Elevator control deflection */
    double  delrx;          /* *io  (d)     Rudder control deflection */
    double  elvlx;          /* *io  (d)     Left elevon deflection */
    double  elvrx;          /* *io  (d)     Right elevon deflection */
    double  elvlcx;         /* *io  (d)     Commanded left elevon deflection */
    double  elvrcx;         /* *io  (d)     Commanded right elevon deflection */
    double  dxd[3];         /* *io  (d/s)   Fin position 1-3 derivative */
    double  dx[3];          /* *io  (d)     Fin position 1-3 */
    double  ddxd[3];        /* *io  (d/s2)  Fin rate derivative 1-3 */
    double  ddx[3];         /* *io  (d/s)   Fin rate 1-3 */
    double  dyd[3];         /* *io  (d/s)   Fin position derivative 4-6 */
    double  dy[3];          /* *io  (d)     Fin position 4-6 */
    double  ddyd[3];        /* *io  (d/s2)  Fin rate derivative 4-6 */
    double  ddy[3];         /* *io  (d/s)   Fin rate 4-6 */
    double  delx1;          /* *io  (d)     Fin 1 deflection */
    double  delx2;          /* *io  (d)     Fin 2 deflection */
    double  delx3;          /* *io  (d)     Fin 3 deflection */
    double  delx4;          /* *io  (d)     Fin 4 deflection */
    double  delx5;          /* *io  (d)     Fin 5 deflection */
    double  delx6;          /* *io  (d)     Fin 6 deflection */
};

#endif  // __ACTUATOR_HH__
