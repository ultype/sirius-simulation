#include <iomanip>

#include "cad/utility.hh"

#include "math/utility.hh"
#include "math/integrate.hh"
#include "math/matrix/utility.hh"

#include "rocket/Kinematics.hh"
#include "sim_services/include/simtime.h"

Kinematics::Kinematics(Newton &newt, Environment &env, _Euler_ &eul)
    :   newton(&newt), euler(&eul), environment(&env),
        MATRIX_INIT(TBD, 3, 3),
        MATRIX_INIT(TBI, 3, 3),
        MATRIX_INIT(TBID, 3, 3),
        VECTOR_INIT(TBI_Q, 4),
        VECTOR_INIT(TBID_Q, 4),
        VECTOR_INIT(TBDQ, 4)
{
    this->default_data();
}

Kinematics::Kinematics(const Kinematics& other)
    :   newton(other.newton), euler(other.euler), environment(other.environment),
        MATRIX_INIT(TBD, 3, 3),
        MATRIX_INIT(TBI, 3, 3),
        MATRIX_INIT(TBID, 3, 3),
        VECTOR_INIT(TBI_Q, 4),
        VECTOR_INIT(TBID_Q, 4),
        VECTOR_INIT(TBDQ, 4)
{
    this->default_data();

    /* Propagative Stats */
    this->TBD = other.TBD;
    this->TBI = other.TBI;
    this->TBID = other.TBID;
    this->TBI_Q = other.TBI_Q;
    this->TBID_Q = other.TBID_Q;
    this->TBDQ = other.TBDQ;

    this->alphax = other.alphax;
    this->betax = other.betax;
    this->alppx = other.alppx;
    this->phipx = other.phipx;
    this->psibdx = other.psibdx;
    this->thtbdx = other.thtbdx;
    this->phibdx = other.phibdx;
    this->ortho_error = other.ortho_error;
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
    this->TBD = other.TBD;
    this->TBI = other.TBI;
    this->TBID = other.TBID;
    this->TBI_Q = other.TBI_Q;
    this->TBID_Q = other.TBID_Q;
    this->alphax = other.alphax;
    this->betax = other.betax;
    this->alppx = other.alppx;
    this->phipx = other.phipx;
    this->psibdx = other.psibdx;
    this->thtbdx = other.thtbdx;
    this->phibdx = other.phibdx;
    this->ortho_error = other.ortho_error;
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

    TBD = build_psi_tht_phi_TM(psibdx * RAD, thtbdx * RAD, phibdx * RAD);

    arma::mat current_TDI = cad::tdi84(lonx * RAD, latx * RAD, alt, get_rettime());
    TBI = TBD * current_TDI;

    this->TBI_Q = Matrix2Quaternion(this->TBI);  //Convert Direct Cosine Matrix to Quaternion
}

void Kinematics::initialize(){
}

void Kinematics::default_data(){
}

void Kinematics::propagate(double int_step){

    double dvba = environment->get_dvba();
    double lonx = newton->get_lonx();
    double latx = newton->get_latx();
    double alt  = newton->get_alt();

    arma::vec WBIB = euler->get_WBIB();
    arma::vec VAED = environment->get_VAED();

    arma::vec VBED = newton->get_VBED();
    arma::vec VBII = newton->get_VBII();

    /* Propagate Quaternion */
    propagate_TBI_Q(int_step, WBIB);

    this->TBD = calculate_TBD(lonx, latx, alt);

    if(newton->get_liftoff()==1){
        arma::vec3 VBAB = TBD * (VBED - VAED);
        this->alphax = calculate_alphax(VBAB);
        this->betax  = calculate_betax(VBAB, dvba);

        this->alppx = calculate_alppx(VBAB, dvba);
        this->phipx = calculate_phipx(VBAB);
    }
    //*incidence angles using wind vector VAED in geodetic coord


    //*diagnostic: calculating the inertial incidence angles
    arma::vec3 VBIB = TBI * VBII;
    this->alphaix = calculate_alphaix(VBIB);
    this->betaix  = calculate_betaix(VBIB);

    this->TBDQ = Matrix2Quaternion(this->TBD);
}


void Kinematics::update_diagnostic_attributes(double int_step) {
    this->thtbdx = get_thtbdx();
    this->psibdx = get_psibdx();
    this->phibdx = get_phibdx();
}

void Kinematics::propagate_TBI(double int_step, arma::vec3 WBIB) {
    //*integrating direction cosine matrix
    INTEGRATE_MAT(TBI, trans(skew_sym(WBIB)) * this->TBI);

    //orthonormalizing TBI
    arma::mat EE = arma::eye(3, 3) - TBI * trans(TBI);
    this->TBI = TBI + EE * TBI * 0.5;

    //TBI orthogonality check
    double e1 = EE(0,0);
    double e2 = EE(1,1);
    double e3 = EE(2,2);
    this->ortho_error = sqrt(e1 * e1 + e2 * e2 + e3 * e3);

}

void Kinematics::propagate_TBI_Q(double int_step, arma::vec3 WBIB)
{
    arma::vec TBID_Q_NEW(4);
    /* Prepare for orthonormalization */
    double quat_metric = TBI_Q(0) * TBI_Q(0) + TBI_Q(1) * TBI_Q(1) + TBI_Q(2) * TBI_Q(2) + TBI_Q(3) * TBI_Q(3);
    double erq = 1. - quat_metric;

    /* Calculate Previous states */
    TBID_Q_NEW(0) = 0.5 * (-WBIB(0) * TBI_Q(1) - WBIB(1) * TBI_Q(2) - WBIB(2) * TBI_Q(3)) + 50. * erq * TBI_Q(0);
    TBID_Q_NEW(1) = 0.5 * (WBIB(0) * TBI_Q(0) + WBIB(2) * TBI_Q(2) - WBIB(1) * TBI_Q(3)) + 50. * erq * TBI_Q(1);
    TBID_Q_NEW(2) = 0.5 * (WBIB(1) * TBI_Q(0) - WBIB(2) * TBI_Q(1) + WBIB(0) * TBI_Q(3)) + 50. * erq * TBI_Q(2);
    TBID_Q_NEW(3) = 0.5 * (WBIB(2) * TBI_Q(0) + WBIB(1) * TBI_Q(1) - WBIB(0) * TBI_Q(2)) + 50. * erq * TBI_Q(3);

    this->TBI_Q = integrate(TBID_Q_NEW, this->TBID_Q, this->TBI_Q, int_step);

    this->TBID_Q = TBID_Q_NEW;

    this->TBI = Quaternion2Matrix(this->TBI_Q);  //Convert Quaternion to Matrix

    //TBI orthogonality check
    arma::mat TIB = trans(TBI);
    arma::mat UBI = TIB * TBI;
    double e1 = UBI(0,0) - 1.;
    double e2 = UBI(1,1) - 1.;
    double e3 = UBI(2,2) - 1.;
    this->ortho_error = sqrt(e1 * e1 + e2 * e2 + e3 * e3);
}

arma::mat Kinematics::calculate_TBD(double lonx, double latx, double alt) {
    //_Euler_ angles
    arma::mat TDI = cad::tdi84(lonx * RAD, latx * RAD, alt, get_rettime());
    return this->TBI * trans(TDI);
}

double Kinematics::calculate_alphaix(arma::vec3 VBIB) {
    return atan2(VBIB(2), VBIB(0)) * DEG;
}

double Kinematics::calculate_betaix(arma::vec3 VBIB) {
    double dvbi = norm(VBIB);
    return asin(VBIB(1) / dvbi) * DEG;
}

double Kinematics::calculate_alppx(arma::vec3 VBAB, double dvba) {
    //incidence angles in load factor plane (aeroballistic)
    double dum = VBAB(0) / dvba;

    if(fabs(dum) > 1)
        dum = 1 * sign(dum);
    double alpp = acos(dum);

    return alpp * DEG;
}

double Kinematics::calculate_phipx(arma::vec3 VBAB) {
    double phip = 0;
    // Changed according to comments, not original code, refer commit:b613a992
    if(VBAB(1) == 0 && VBAB(2) == 0){
        phip = 0.;
    }else if(fabs(VBAB(1)) < arma::datum::eps){
        //note: if vbeb2 is <EPS the value phip is forced to be 0 or PI
        //      to prevent oscillations
        if(VBAB(3) > 0) phip = 0;
        if(VBAB(3) < 0) phip = PI;
    }
    else{
        phip = atan2(VBAB(1), VBAB(2));
    }

    return phip * DEG;
}

double Kinematics::calculate_alphax(arma::vec3 VBAB) {
    double alpha = atan2(VBAB(2), VBAB(0));
    return alpha * DEG;
}

double Kinematics::calculate_betax(arma::vec3 VBAB, double dvba) {
    double beta = asin(VBAB(1) / dvba);
    return beta * DEG;
}

double Kinematics::get_alppx() { return alppx; }
double Kinematics::get_phipx() { return phipx; }

double Kinematics::get_alphax() { return alphax; }
double Kinematics::get_betax() { return betax; }

double Kinematics::get_psibdx() {
    
    double cthtbd = 0;

    get_thtbdx_in(cthtbd);

    //yaw angle: 'psibd'
    double cpsi = TBD(0, 0) / cthtbd;
    if(fabs(cpsi) > 1){
        cpsi = 1 * sign(cpsi);
    }
    psibd = acos(cpsi) * sign(TBD(0, 1));
    // psibd = atan2( 2. * (TBDQ(1) * TBDQ(2) + TBDQ(0) * TBDQ(3) ), (1. - 2. * (TBDQ(2) * TBDQ(2) + TBDQ(3) * TBDQ(3))) ); 

    return DEG * psibd;
}

double Kinematics::get_thtbdx() {
    double cthtbd = 0;
    return get_thtbdx_in(cthtbd);
}

double Kinematics::get_thtbdx_in(double &cthtbd) {
    

    //*geodetic _Euler_ angles
    //pitch angle: 'thtbd'
    //note: when |tbd13| >= 1, thtbd = +- pi/2, but cos(thtbd) is
    //      forced to be a small positive number to prevent division by zero
    if(fabs(TBD(0, 2)) < 1){
        thtbd = asin(-TBD(0, 2));
        cthtbd = cos(thtbd);
    }else{
        thtbd = PI / 2 * sign(-TBD(0, 2));
        cthtbd = arma::datum::eps;
    }
    // thtbd = asin(  2 * (TBDQ(0) * TBDQ(2) - TBDQ(1) * TBDQ(3)));

    return DEG * thtbd;
}
double Kinematics::get_phibdx() {
    
    double cthtbd = 0;

    get_thtbdx_in(cthtbd);

    //roll angle: 'phibdc'
    double cphi = TBD(2, 2) / cthtbd;
    if(fabs(cphi) > 1){
        cphi = 1 * sign(cphi);
    }
    phibd = acos(cphi) * sign(TBD(1, 2));
    //phibd = atan2( 2. * (TBDQ(2) * TBDQ(3) + TBDQ(0) * TBDQ(1) ), (1. - 2. * (TBDQ(2) * TBDQ(2) + TBDQ(1) * TBDQ(1))));


    return DEG * phibd;
}

arma::mat Kinematics::get_TBD() { return TBD; }

arma::mat Kinematics::get_TBI() { return TBI; }
