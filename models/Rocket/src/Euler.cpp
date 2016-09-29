#include "Euler.hh"
#include "trick_utils/math/include/quat_macros.h"
#include "trick_utils/math/include/matrix_macros.h"
#include "trick_utils/math/include/vector_macros.h"
#include "trick_utils/math/include/trick_math_proto.h"
#include "sim_services/include/simtime.h"

void _Euler_::initialization(Kinematics* kine, Propulsion* prop, Forces* forc)
{
    kinematics=kine;
    propulsion=prop;
    force=forc;

    Matrix TBI(3,3);
    Matrix WBEB(3,1);
    Matrix WBIB(3,1);
    Matrix WEII(3,1);

    TBI.build_mat33(kinematics->tbi[0][0],kinematics->tbi[0][1],kinematics->tbi[0][2],kinematics->tbi[1][0], kinematics->tbi[1][1],kinematics->tbi[1][2],
                    kinematics->tbi[2][0],kinematics->tbi[2][1],kinematics->tbi[2][2]);
    //body rate wrt Earth frame in body coordinates
    WBEB.build_vec3(ppx*RAD,qqx*RAD,rrx*RAD);
    //body rate wrt ineritial frame in body coordinates
    WEII.build_vec3(0,0,WEII3);
    WBIB=WBEB+TBI*WEII;
    //update martrix
    WBIB.fill(wbib);
}


void _Euler_::euler(double int_step)
{
    Matrix WEII(3,1);
    Matrix WBEB(3,1);
    Matrix WBII(3,1);
    Matrix FMB(3,1);
    Matrix IBBB(3,3);
    Matrix WBIBD(3,1);
    Matrix TBI(3,3);
    Matrix WBIB(3,1);

    //input data from other module
    TBI.build_mat33(kinematics->tbi[0][0],kinematics->tbi[0][1],kinematics->tbi[0][2],
                        kinematics->tbi[1][0],kinematics->tbi[1][1],kinematics->tbi[1][2],
                        kinematics->tbi[2][0],kinematics->tbi[2][1],kinematics->tbi[2][2]);
    //body rate wrt Earth frame in body coordinates
    WBEB.build_vec3(ppx*RAD,qqx*RAD,rrx*RAD);
    //body rate wrt ineritial frame in body coordinates
    WEII.build_vec3(0,0,WEII3);
    FMB.build_vec3(force->fmb[0],force->fmb[1],force->fmb[2]);
    IBBB.build_mat33(propulsion->ibbb[0][0],propulsion->ibbb[0][1],propulsion->ibbb[0][2],
                        propulsion->ibbb[1][0],propulsion->ibbb[1][1],propulsion->ibbb[1][2],
                        propulsion->ibbb[2][0],propulsion->ibbb[2][1],propulsion->ibbb[2][2]);
    WBIB.build_vec3(wbib[0],wbib[1],wbib[2]);
    /***********************************************************************************************/
    //integrating the angular velocity acc wrt the inertial frame in body coord

    Matrix WACC_NEXT=IBBB.inverse()*(FMB-WBIB.skew_sym()*IBBB*WBIB);
    WBIB=integrate(WACC_NEXT,WBIBD,WBIB,int_step);
    WBIBD=WACC_NEXT;

    //angular velocity wrt inertial frame in inertial coordinates
    WBII=~TBI*WBIB;

    //angular velocity wrt Earth in body coordinates
    WBEB=WBIB-TBI*WEII;

    //body rates in deg/s
    ppx=WBEB.get_loc(0,0)*DEG;
    qqx=WBEB.get_loc(1,0)*DEG;
    rrx=WBEB.get_loc(2,0)*DEG;

    //update matrix elements to module variables
    WBIB.fill(wbib);
    WBIBD.fill(wbibd);
    WBEB.fill(wbeb);
    WBII.fill(wbii);
}
