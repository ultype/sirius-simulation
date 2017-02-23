#include <iomanip>

#include "rocket/Kinematics.hh"
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

    TBD = build_euler_transform_matrix(psibdx * RAD, thtbdx * RAD, phibdx * RAD);

    arma::mat current_TDI = arma_cad_tdi84(lonx * RAD, latx * RAD, alt, get_rettime());
    TBI = TBD * current_TDI;

    arma::vec tbid_q(4);
    tbid_q(0) = 0.0;
    tbid_q(1) = 0.0;
    tbid_q(2) = 0.0;
    tbid_q(3) = 0.0;
    this->TBI_Q = Matrix2Quaternion(this->TBI);
    this->TBID_Q = tbid_q;

}

void Kinematics::initialize(){
}

void Kinematics::default_data(){
}

void Kinematics::propagate(double int_step){
    double cthtbd = 0;
    double phip = 0;

    double dvba = environment->get_dvba();
    double lonx = newton->get_lonx();
    double latx = newton->get_latx();
    double alt  = newton->get_alt();

    arma::vec WBIB = euler->get_WBIB_();
    arma::vec VAED = arma::vec3(environment->get_VAED().get_pbody());

    arma::vec VBED = newton->get_VBED_();
    arma::vec VBII = newton->get_VBII();

    propagate_TBI(int_step, WBIB);
    //propagate_TBI_Q(int_step, WBIB);
    //this->TBI = Quaternion2Matrix(this->TBI_Q);
    this->TBD = calculate_TBD(lonx, latx, alt);

    //*incidence angles using wind vector VAED in geodetic coord
    arma::vec3 VBAB = TBD * (VBED - VAED);
    this->alphax = calculate_alphax(VBAB);
    this->betax  = calculate_betax(VBAB, dvba);

    this->alppx = calculate_alppx(VBAB, dvba);
    this->phipx = calculate_phipx(VBAB);

    //*diagnostic: calculating the inertial incidence angles
    arma::vec3 VBIB = TBI * VBII;
    this->alphaix = calculate_alphaix(VBIB);
    this->betaix  = calculate_betaix(VBIB);
}


void Kinematics::update_diagnostic_attributes(double int_step) {
    this->thtbdx = get_thtbdx();
    this->psibdx = get_psibdx();
    this->phibdx = get_phibdx();
}

void Kinematics::propagate_TBI(double int_step, arma::vec3 WBIB) {
    //*integrating direction cosine matrix
    arma::mat TBID_NEW = trans(skew_sym(WBIB)) * this->TBI;
    this->TBI = integrate(TBID_NEW, TBID, TBI, int_step);
    this->TBID = TBID_NEW;

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
    double quat_metric = TBI_Q(0) * TBI_Q(0) + TBI_Q(1) * TBI_Q(1) +TBI_Q(2) * TBI_Q(2) +TBI_Q(3) * TBI_Q(3);
    double erq = 1. -quat_metric;

    TBID_Q_NEW(0) = 0.5 * (-WBIB(0) * TBI_Q(1) - WBIB(1) * TBI_Q(2) - WBIB(2) * TBI_Q(3)) + 50. * erq * TBI_Q(0);
    TBID_Q_NEW(1) = 0.5 * (WBIB(0) * TBI_Q(0) + WBIB(2) * TBI_Q(2) - WBIB(1) * TBI_Q(3)) + 50. * erq * TBI_Q(1);
    TBID_Q_NEW(2) = 0.5 * (WBIB(1) * TBI_Q(0) - WBIB(2) * TBI_Q(1) + WBIB(0) * TBI_Q(3)) + 50. * erq * TBI_Q(2);
    TBID_Q_NEW(3) = 0.5 * (WBIB(2) * TBI_Q(0) + WBIB(1) * TBI_Q(1) - WBIB(0) * TBI_Q(2)) + 50. * erq * TBI_Q(3);

    TBI_Q = integrate(TBID_Q_NEW, TBID_Q, TBI_Q, int_step);

    this->TBID_Q = TBID_Q_NEW;

}

arma::mat Kinematics::calculate_TBD(double lonx, double latx, double alt) {
    //_Euler_ angles
    arma::mat TDI = arma_cad_tdi84(lonx * RAD, latx * RAD, alt, get_rettime());
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
    double psibd = 0;
    double cthtbd = 0;

    get_thtbdx(cthtbd);

    //yaw angle: 'psibd'
    double cpsi = TBD(0, 0) / cthtbd;
    if(fabs(cpsi) > 1){
        cpsi = 1 * sign(cpsi);
    }
    psibd = acos(cpsi) * sign(TBD(0, 1));

    return DEG * psibd;
}

double Kinematics::get_thtbdx() {
    double cthtbd = 0;
    return get_thtbdx(cthtbd);
}

double Kinematics::get_thtbdx(double &cthtbd) {
    double thtbd = 0;

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

    return DEG * thtbd;
}
double Kinematics::get_phibdx() {
    double phibd = 0;
    double cthtbd = 0;

    get_thtbdx(cthtbd);

    //roll angle: 'phibdc'
    double cphi = TBD(2, 2) / cthtbd;
    if(fabs(cphi) > 1){
        cphi = 1 * sign(cphi);
    }
    phibd = acos(cphi) * sign(TBD(1, 2));

    return DEG * phibd;
}

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
