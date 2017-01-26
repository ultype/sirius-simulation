#include "rocket/Euler.hh"
#include "trick_utils/math/include/quat_macros.h"
#include "trick_utils/math/include/matrix_macros.h"
#include "trick_utils/math/include/vector_macros.h"
#include "trick_utils/math/include/trick_math_proto.h"
#include "sim_services/include/simtime.h"

_Euler_::_Euler_(Kinematics& kine, Propulsion& prop, Forces& forc)
    :   kinematics(&kine), propulsion(&prop), forces(&forc),
        VECTOR_INIT(WEII, 3),
        VECTOR_INIT(WBII, 3),
        VECTOR_INIT(WBIB, 3),
        VECTOR_INIT(WBIBD, 3),
        VECTOR_INIT(WBEB, 3)
{
    this->default_data();
}

_Euler_::_Euler_(const _Euler_& other)
    :   kinematics(other.kinematics), propulsion(other.propulsion), forces(other.forces),
        VECTOR_INIT(WEII, 3),
        VECTOR_INIT(WBII, 3),
        VECTOR_INIT(WBIB, 3),
        VECTOR_INIT(WBIBD, 3),
        VECTOR_INIT(WBEB, 3)
{
    this->default_data();

    /* Propagative Stats */
}

_Euler_& _Euler_::operator=(const _Euler_& other){
    if(&other == this)
        return *this;

    this->kinematics = other.kinematics;
    this->propulsion = other.propulsion;
    this->forces = other.forces;

    /* Propagative Stats */

    return *this;
}

void _Euler_::load_angular_velocity(double ppx, double qqx, double rrx){
    arma::mat33 TBI = kinematics->get_TBI_();

    //body rate wrt Earth frame in body coordinates
    WBEB = {ppx * RAD, qqx * RAD, rrx * RAD};
    WBIB = WBEB + TBI * WEII;
}

void _Euler_::initialize()
{
}

void _Euler_::default_data(){
    WEII.zeros();
    WEII(2) = WEII3;
}

void _Euler_::euler(double int_step)
{
    arma::vec3 FMB(forces->get_FMB().get_pbody());

    // XXX: Trans
    arma::mat33 IBBB = (propulsion->get_IBBB().get_pbody());
    IBBB = trans(IBBB);

    arma::mat33 TBI = kinematics->get_TBI_();

    //body rate wrt Earth frame in body coordinates
    WBEB = {ppx * RAD, qqx * RAD,rrx * RAD};
    //body rate wrt ineritial frame in body coordinates

    //integrating the angular velocity acc wrt the inertial frame in body coord
    // Using Armadillo solve for higher accuracy, otherwise will faile the 1ppm test
    arma::vec3 WACC_NEXT = arma::solve(IBBB, (FMB - skew_sym(WBIB) * IBBB * WBIB));
    WBIB = integrate(WACC_NEXT, WBIBD, WBIB, int_step);
    WBIBD = WACC_NEXT;

    //angular velocity wrt inertial frame in inertial coordinates
    WBII = trans(TBI) * WBIB;

    //angular velocity wrt Earth in body coordinates
    WBEB = WBIB - TBI * WEII;

    //body rates in deg/s
    ppx = WBEB(0) * DEG;
    qqx = WBEB(1) * DEG;
    rrx = WBEB(2) * DEG;

}

double _Euler_::get_ppx() { return ppx; }

double _Euler_::get_qqx() { return qqx; }

double _Euler_::get_rrx() { return rrx; }

Matrix _Euler_::get_WBII()
{
    Matrix WBII(_WBII);
    return WBII;
}

Matrix _Euler_::get_WBIB()
{
    Matrix WBIB(_WBIB);
    return WBIB;
}

