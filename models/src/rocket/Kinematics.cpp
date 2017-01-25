#include <iomanip>

#include "rocket/Kinematics.hh"
#include "trick_utils/math/include/quat_macros.h"
#include "trick_utils/math/include/matrix_macros.h"
#include "trick_utils/math/include/vector_macros.h"
#include "trick_utils/math/include/trick_math_proto.h"
#include "sim_services/include/simtime.h"

Kinematics::Kinematics(Newton &newt, Environment &env, _Euler_ &eul)
    :   newton(&newt), euler(&eul), environment(&env),
        MATRIX_INIT(TBD, 3, 3),
        MATRIX_INIT(TBI, 3, 3),
        MATRIX_INIT(TBID, 3, 3)
{
    this->default_data();
}

Kinematics::Kinematics(const Kinematics& other)
    :   newton(other.newton), euler(other.euler), environment(other.environment),
        MATRIX_INIT(TBD, 3, 3),
        MATRIX_INIT(TBI, 3, 3),
        MATRIX_INIT(TBID, 3, 3)
{
    this->default_data();

    /* Propagative Stats */
    //memcpy(this->tbd, other.tbd, sizeof(tbd));
    //memcpy(this->tbi, other.tbi, sizeof(tbd));
    //memcpy(this->tbid, other.tbid, sizeof(tbd));
    this->TBD = other.TBD;
    this->TBI = other.TBI;
    this->TBID = other.TBID;

    this->alphax = other.alphax;
    this->betax = other.betax;
    this->alppx = other.alppx;
    this->phipx = other.phipx;
    this->psibdx = other.psibdx;
    this->thtbdx = other.thtbdx;
    this->phibdx = other.phibdx;
    this->ortho_error = other.ortho_error;
    this->psibd = other.psibd;
    this->thtbd = other.thtbd;
    this->phibd = other.phibd;
    this->alphaix = other.alphaix;
    this->betaix = other.betaix;
}

Kinematics& Kinematics::operator=(const Kinematics& other){
    if(&other == this)
        return *this;

    this->newton = other.newton;
    this->environment = other.environment;
    this->euler = other.euler;

    /* Propagative Stats */
    //memcpy(this->tbd, other.tbd, sizeof(tbd));
    //memcpy(this->tbi, other.tbi, sizeof(tbd));
    //memcpy(this->tbid, other.tbid, sizeof(tbd));
    this->TBD = other.TBD;
    this->TBI = other.TBI;
    this->TBID = other.TBID;
    this->alphax = other.alphax;
    this->betax = other.betax;
    this->alppx = other.alppx;
    this->phipx = other.phipx;
    this->psibdx = other.psibdx;
    this->thtbdx = other.thtbdx;
    this->phibdx = other.phibdx;
    this->ortho_error = other.ortho_error;
    this->psibd = other.psibd;
    this->thtbd = other.thtbd;
    this->phibd = other.phibd;
    this->alphaix = other.alphaix;
    this->betaix = other.betaix;

    return *this;
}

void Kinematics::load_angle(double yaw, double roll, double pitch) {
    this->psibdx = yaw;
    this->phibdx = roll;
    this->thtbdx = pitch;

    double lonx = newton->get_lonx();
    double latx = newton->get_latx();
    double alt  = newton->get_alt();

    TBD = build_euler_transform_matrix(psibdx * RAD, thtbdx * RAD, phibdx * RAD);

    arma::mat current_TDI = arma_cad_tdi84(lonx * RAD, latx * RAD, alt, get_rettime());
    TBI = TBD * current_TDI;

}

void Kinematics::initialize(){
}

void Kinematics::default_data(){
}

void Kinematics::calculate_kinematics(double int_step){
    double cthtbd = 0;
    double phip = 0;

    double dvba = environment->get_dvba();
    double lonx = newton->get_lonx();
    double latx = newton->get_latx();
    double alt  = newton->get_alt();

    arma::vec WBIB = arma::vec3(euler->get_WBIB().get_pbody());
    arma::vec VAED = arma::vec3(environment->get_VAED().get_pbody());

    arma::vec VBED = newton->get_VBED_();
    arma::vec VBII = newton->get_VBII();

    //*integrating direction cosine matrix
    arma::mat TBID_NEW = trans(skew_sym(WBIB)) * TBI;
    TBI = integrate(TBID_NEW, TBID, TBI, int_step);
    TBID = TBID_NEW;

    //orthonormalizing TBI
    arma::mat EE = arma::eye(3, 3) - TBI * trans(TBI);
    TBI = TBI + EE * TBI * 0.5;

    //TBI orthogonality check
    double e1 = EE(0,0);
    double e2 = EE(1,1);
    double e3 = EE(2,2);
    ortho_error=sqrt(e1*e1+e2*e2+e3*e3);

    //_Euler_ angles
    arma::mat TDI = arma_cad_tdi84(lonx * RAD, latx * RAD, alt, get_rettime());
    TBD = TBI * trans(TDI);
    double tbd13 = TBD(0,2);
    double tbd11 = TBD(0,0);
    double tbd33 = TBD(2,2);
    double tbd12 = TBD(0,1);
    double tbd23 = TBD(1,2);

    //*geodetic _Euler_ angles
    //pitch angle: 'thtbd'
    //note: when |tbd13| >= 1, thtbd = +- pi/2, but cos(thtbd) is
    //      forced to be a small positive number to prevent division by zero
    if(fabs(tbd13) < 1){
        thtbd = asin(-tbd13);
        cthtbd = cos(thtbd);
    }else{
        thtbd = PI / 2 * sign(-tbd13);
        cthtbd = arma::datum::eps;
    }

    //yaw angle: 'psibd'
    double cpsi = tbd11 / cthtbd;
    if(fabs(cpsi) > 1){
        cpsi = 1 * sign(cpsi);
    }
    psibd = acos(cpsi) * sign(tbd12);

    //roll angle: 'phibdc'
    double cphi = tbd33 / cthtbd;
    if(fabs(cphi) > 1){
        cphi = 1 * sign(cphi);
    }
    phibd = acos(cphi) * sign(tbd23);

    psibdx = DEG * psibd;
    thtbdx = DEG * thtbd;
    phibdx = DEG * phibd;

    //*incidence angles using wind vector VAED in geodetic coord
    arma::vec3 VBAB = TBD * (VBED - VAED);
    double vbab1 = VBAB(0);
    double vbab2 = VBAB(1);
    double vbab3 = VBAB(2);
    double alpha = atan2(vbab3, vbab1);
    double beta = asin(vbab2 / dvba);
    alphax = alpha * DEG;
    betax = beta * DEG;

    //incidence angles in load factor plane (aeroballistic)
    double dum = vbab1 / dvba;

    if(fabs(dum) > 1)
        dum = 1 * sign(dum);
    double alpp = acos(dum);

    //XXX: WTF over this place? Is this correct? Code differ from comments
    if(vbab2 == 0 && vbab3 == 0)
        phip = 0.;
    else if(fabs(vbab2) < arma::datum::eps)
        //note: if vbeb2 is <EPS the value phip is forced to be 0 or PI
        //      to prevent oscillations
        // XXX: Missing Braces?
        if(vbab3 > 0) phip = 0;
    if(vbab3 < 0) phip = PI;
    else
        phip=atan2(vbab2,vbab3);
    alppx = alpp * DEG;
    phipx = phip * DEG;

    //*diagnostic: calculating the inertial incidence angles
    arma::vec3 VBIB = TBI * VBII;
    double vbib1 = VBIB(0);
    double vbib2 = VBIB(1);
    double vbib3 = VBIB(2);
    double alphai = atan2(vbib3, vbib1);
    double dvbi = norm(VBIB);
    double betai = asin(vbib2 / dvbi);
    alphaix = alphai * DEG;
    betaix = betai*DEG;

}

double Kinematics::get_alppx() { return alppx; }
double Kinematics::get_phipx() { return phipx; }
double Kinematics::get_alphax() { return alphax; }
double Kinematics::get_betax() { return betax; }
double Kinematics::get_psibdx() { return psibdx; }
double Kinematics::get_thtbdx() { return thtbdx; }
double Kinematics::get_phibdx() { return phibdx; }

Matrix Kinematics::get_TBD() {
    Matrix TBD(3, 3);
    TBD.build_mat33(_TBD);
    return ~TBD;
}

Matrix Kinematics::get_TBI() {
    Matrix TBI(3, 3);
    TBI.build_mat33(_TBI);
    return ~TBI;
}

arma::mat Kinematics::get_TBD_() {
    return TBD;
}

arma::mat Kinematics::get_TBI_() {
    return TBI;
}
