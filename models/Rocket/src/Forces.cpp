#include "forces.hh"
#include "trick_utils/math/include/quat_macros.h"
#include "trick_utils/math/include/matrix_macros.h"
#include "trick_utils/math/include/vector_macros.h"
#include "trick_utils/math/include/trick_math_proto.h"
#include "sim_services/include/simtime.h"
void Forces::init_force(Environment* env, Propulsion* prop, RCS* rcs
    , AeroDynamic* aero, TVC* tvc)
{
    Environment=env;
    Propulsion=prop;
    RCS=rcs;
    Aerodynamic=aero;
    TVC=tvc;
}
void Forces::forces()
{


    /*****************input from another module*******************/
    double pdynmc=Environment->pdynmc;
    double mprop=Propulsion->mprop;
    double thrust=Propulsion->thrust;
    double mrcs_moment=RCS->mrcs_moment;
    double mrcs_force=RCS->mrcs_force;
    double refa=Aerodynamic->refa;
    double refd=Aerodynamic->refd;
    double cy=Aerodynamic->cy;
    double cll=Aerodynamic->cll;
    double clm=Aerodynamic->clm;
    double cln=Aerodynamic->cln;
    double cx=Aerodynamic->cx;
    double cz=Aerodynamic->cz;
    int mtvc=TVC->mtvc;

    Matrix FMRCS(3,1);
    Matrix FARCS(3,1);
    Matrix FPB(3,1);
    Matrix FMPB(3,1);

    FMRCS.build_vec3(RCS->fmrcs[0],RCS->fmrcs[1],RCS->fmrcs[2]);
    FARCS.build_vec3(RCS->farcs[0],RCS->farcs[1],RCS->farcs[2]);
    FPB.build_vec3(TVC->fpb[0],TVC->fpb[1],TVC->fpb[2]);
    FMPB.build_vec3(TVC->fmpb[0],TVC->fmpb[1],TVC->fmpb[2]);
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
    for(i=0;i<3;i++){
        fapb[i]=FAPB.get_loc(i,0);
    }
    for(i=0;i<3;i++){
        fmb[i]=FMB.get_loc(i,0);
    }
    for(i=0;i<3;i++){
        fap[i]=FAP.get_loc(i,0);
    }
    /*******************************************/
}
