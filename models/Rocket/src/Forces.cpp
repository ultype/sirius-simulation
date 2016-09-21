#include "Force.hh"
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
    double pdynmc=environment->pdynmc;
    double mprop=propulsion->mprop;
    double thrust=propulsion->thrust;
    double mrcs_moment=rcs->mrcs_moment;
    double mrcs_force=rcs->mrcs_force;
    double refa=Aerodynamics->refa;
    double refd=Aerodynamics->refd;
    double cy=Aerodynamics->cy;
    double cll=Aerodynamics->cll;
    double clm=Aerodynamics->clm;
    double cln=Aerodynamics->cln;
    double cx=Aerodynamics->cx;
    double cz=Aerodynamics->cz;
    int mtvc=tvc->mtvc;

    Matrix FMRCS(3,1);
    Matrix FARCS(3,1);
    Matrix FPB(3,1);
    Matrix FMPB(3,1);
    Matrix FAPB(3,1);
    Matrix FAP(3,1);
    Matrix FMB(3,1);

    FMRCS.build_vec3(rcs->fmrcs[0],rcs->fmrcs[1],rcs->fmrcs[2]);
    FARCS.build_vec3(rcs->farcs[0],rcs->farcs[1],rcs->farcs[2]);
    FPB.build_vec3(tvc->fpb[0],tvc->fpb[1],tvc->fpb[2]);
    FMPB.build_vec3(tvc->fmpb[0],tvc->fmpb[1],tvc->fmpb[2]);
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
