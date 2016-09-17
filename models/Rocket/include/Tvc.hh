#ifndef __TVC_HH__
#define __TVC_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the TVC Module On Board)
LIBRARY DEPENDENCY:
      ((../src/Tvc.cpp))
PROGRAMMERS:
      (((Chung-Fan Yang) () () () ))
*******************************************************************************/

#include "Newton.hh"
#include "Euler.hh"
#include "Environment.hh"
#include "Kinematics.hh"
#include "GPS_receiver.hh"
#include "Control.hh"

class TVC {
    public:
    TVC() {}

    void default_data();
    void initialize(Environment *env, Kinematics *kins, Control *con, Propulsion *plp);

    void actuate(double int_step);
    void tvc_scnd(double &eta, double &zet, double etac, double zetc, double int_step);

    Environment *environment;
    Kinematics *kinematics;
    Control *control;
    Propulsion *propulsion;

    int mtvc;            /* *io  (--)    =0:no TVC;=1:no dyn;=2:scnd order;=3:2+gain */
    double tvclimx;     /* *io  (d)     Nozzle deflection limiter */
    double dtvclimx;    /* *io  (d/s)   Nozzle deflection rate limiter */
    double wntvc;       /* *io  (r/s)   Natural frequency of TVC */
    double zettvc;      /* *io  (--)    Damping of TVC */
    double factgtvc;    /* *io  (--)    Factor for TVC gain */
    double gtvc;        /* *io  (--)    TVC nozzle deflection gain */
    double parm;        /* *io  (m)     Propulsion moment arm from vehicle nose */
    double fpb[3];      /* *io  (N)     Thrust force in body axes */
    double fmpb[3];     /* *io  (N*m)    Thrust moment in body axes */
    double etax;        /* *io  (d)     Nozzle pitch deflection */
    double zetx;        /* *io  (d)     Nozzle yaw deflection */
    double etacx;       /* *io  (d)     Commanded nozzle pitch deflection */
    double zetcx;       /* *io  (d)     Commanded nozzle yaw deflection */
    double etasd;       /* *io  (r/s)   Pitch nozzle derivative */
    double zetad;       /* *io  (r/s)   Yaw nozzle derivative */
    double etas;        /* *io  (r)     Pitch nozzle deflection */
    double zeta;        /* *io  (r)     Yaw nozzle deflection */
    double detasd;      /* *io  (r/s2)  Pitch nozzle rate derivative */
    double dzetad;      /* *io  (r/s2)  Yaw nozzle rate derivative */
    double detas;       /* *io  (r/s)   Pitch nozzle rate */
    double dzeta;       /* *io  (r/s)   Yaw nozzle rate */
};

#endif  // __TVC_HH__
