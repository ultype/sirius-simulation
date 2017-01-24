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

    Matrix TBI = kinematics->get_TBI();
    Matrix WBEB(3,1);
    Matrix WBIB(3,1);
    Matrix WEII(3,1);

    //body rate wrt Earth frame in body coordinates
    WBEB.build_vec3(ppx*RAD,qqx*RAD,rrx*RAD);
    //body rate wrt ineritial frame in body coordinates
    WEII.build_vec3(0,0,WEII3);
    WBIB=WBEB+TBI*WEII;
    //update martrix
    WBIB.fill(wbib);
}

double _Euler_::get_ppx() { return ppx; }

double _Euler_::get_qqx() { return qqx; }

double _Euler_::get_rrx() { return rrx; }

Matrix _Euler_::get_WBII()
{
    Matrix WBII(3, 1);
    WBII.build_vec3(wbii);
    return WBII;
}

Matrix _Euler_::get_WBIB()
{
    Matrix WBIB(3, 1);
    WBIB.build_vec3(wbib);
    return WBIB;
}

void _Euler_::euler(double int_step)
{
    Matrix WEII(3,1);
    Matrix WBEB(3,1);
    Matrix WBII(3,1);
    Matrix FMB = force->get_FMB();
    Matrix IBBB(3,3);
    Matrix WBIBD(3,1);
    Matrix TBI = kinematics->get_TBI();
    Matrix WBIB(3,1);

    //body rate wrt Earth frame in body coordinates
    WBEB.build_vec3(ppx*RAD,qqx*RAD,rrx*RAD);
    //body rate wrt ineritial frame in body coordinates
    WEII.build_vec3(0,0,WEII3);
    IBBB.build_mat33(propulsion->ibbb);
    WBIB.build_vec3(wbib);
    WBIBD.build_vec3(wbibd);
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
