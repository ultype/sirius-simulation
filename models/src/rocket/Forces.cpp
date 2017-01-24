#include "rocket/Force.hh"
#include "trick_utils/math/include/quat_macros.h"
#include "trick_utils/math/include/matrix_macros.h"
#include "trick_utils/math/include/vector_macros.h"
#include "trick_utils/math/include/trick_math_proto.h"
#include "sim_services/include/simtime.h"
void Forces::init_force(Environment* env, Propulsion* prop, RCS* rc
    , AeroDynamics* aero, TVC* tv)
{
    environment=env;
    propulsion=prop;
    rcs=rc;
    Aerodynamics=aero;
    tvc=tv;
}

void Forces::forces()
{


    /*****************input from another module*******************/
    double pdynmc=environment->get_pdynmc();
    double mprop=propulsion->get_mprop();
    double thrust=propulsion->get_thrust();
    double mrcs_moment=rcs->get_mrcs_moment();
    double mrcs_force=rcs->get_mrcs_force();
    double refa=Aerodynamics->get_refa();
    double refd=Aerodynamics->get_refd();
    double cy=Aerodynamics->get_cy();
    double cll=Aerodynamics->get_cll();
    double clm=Aerodynamics->get_clm();
    double cln=Aerodynamics->get_cln();
    double cx=Aerodynamics->get_cx();
    double cz=Aerodynamics->get_cz();
    int mtvc=tvc->get_mtvc();

    Matrix FMRCS = rcs->get_FMRCS();
    Matrix FARCS = rcs->get_FARCS();
    Matrix FPB = tvc->get_FPB();
    Matrix FMPB = tvc->get_FMPB();
    Matrix FAPB(3,1);
    Matrix FAP(3,1);
    Matrix FMB(3,1);

    /*************************************************************/

    //total non-gravitational forces
    FAPB.assign_loc(0,0,pdynmc*refa*cx);
    FAPB.assign_loc(1,0,pdynmc*refa*cy);
    FAPB.assign_loc(2,0,pdynmc*refa*cz);
    //aerodynamic loss
    FAP.assign_loc(0,0,pdynmc*refa*cx);
    FAP.assign_loc(1,0,pdynmc*refa*cy);
    FAP.assign_loc(2,0,pdynmc*refa*cz);

    //aerodynamic moment
    FMB.assign_loc(0,0,pdynmc*refa*refd*cll);
    FMB.assign_loc(1,0,pdynmc*refa*refd*clm);
    FMB.assign_loc(2,0,pdynmc*refa*refd*cln);

    //adding thrust modified by TVC or otherwise just plain thrust if mprop > 0
    if(mtvc==1||mtvc==2||mtvc==3){
        FAPB=FAPB+FPB;
        FMB=FMB+FMPB;
    }
    else if(mprop)
        FAPB[0]=FAPB[0]+thrust;

    //adding force components from RCS
    if(mrcs_force==1||mrcs_force==2)
        FAPB=FAPB+FARCS;

    //adding moment components from RCS
    if(mrcs_moment>0&&mrcs_moment<=23)
        FMB=FMB+FMRCS;


    /***********update matrix variable***********/
    FAPB.fill(fapb);
    FMB.fill(fmb);
    FAP.fill(fap);
    /*******************************************/
}

double* Forces::get_fapb_ptr() { return fapb; }
double* Forces::get_fap_ptr() { return fap; }

Matrix Forces::get_FAPB() {
    Matrix FAPB(3,1);
    FAPB.build_vec3(fapb);
    return FAPB;
}

Matrix Forces::get_FAP() {
    Matrix FAP(3,1);
    FAP.build_vec3(fap);
    return FAP;
}

Matrix Forces::get_FMB() {
    Matrix FMB(3,1);
    FMB.build_vec3(fmb);
    return FMB;
}
