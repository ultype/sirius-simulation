#include <iomanip>
#include "cad_utility.hh"
#include "math_utility.hh"
#include "integrate.hh"
#include "matrix/utility.hh"
#include "Rocket_Flight_DM.hh"
#include "sim_services/include/simtime.h"
#include "aux.hh"
#include <tuple>

Rocket_Flight_DM::Rocket_Flight_DM()
    :   MATRIX_INIT(TBD, 3, 3),
        MATRIX_INIT(TBI, 3, 3),
        MATRIX_INIT(TBID, 3, 3),
        MATRIX_INIT(WEII_skew, 3, 3),
        MATRIX_INIT(TDI, 3, 3),
        MATRIX_INIT(TDE, 3, 3),
        MATRIX_INIT(TGI, 3, 3),
        VECTOR_INIT(TBI_Q, 4),
        VECTOR_INIT(TBID_Q, 4),
        VECTOR_INIT(TBDQ, 4),
        VECTOR_INIT(VBAB, 3),
        VECTOR_INIT(SBII, 3),
        VECTOR_INIT(VBII, 3),
        VECTOR_INIT(SBIIP, 3),
        VECTOR_INIT(VBIIP, 3),
        VECTOR_INIT(ABII, 3),
        VECTOR_INIT(FSPB, 3),
        VECTOR_INIT(ABIB, 3),
        VECTOR_INIT(SBEE, 3),
        VECTOR_INIT(VBEE, 3),
        VECTOR_INIT(ABEE, 3),
        VECTOR_INIT(JBII, 3),
        VECTOR_INIT(JBEE, 3),
        VECTOR_INIT(CONING, 3),
        VECTOR_INIT(NEXT_ACC, 3),
        VECTOR_INIT(VBED, 3),
        VECTOR_INIT(VBII_old, 3),
        VECTOR_INIT(WEII, 3),
        VECTOR_INIT(WBII, 3),
        VECTOR_INIT(WBIB, 3),
        VECTOR_INIT(WBIBD, 3),
        VECTOR_INIT(WBEB, 3),
        VECTOR_INIT(SBEE_old, 3),
        VECTOR_INIT(VBEE_old, 3),
        VECTOR_INIT(ABEE_old, 3),
        VECTOR_INIT(SBEE_test, 3),
        VECTOR_INIT(VBEE_test, 3),
        VECTOR_INIT(ABEE_test, 3),
        MATRIX_INIT(TLI, 3, 3),
        VECTOR_INIT(LT_euler, 3),
        VECTOR_INIT(TBLQ, 4) {
            build_WEII();
        }

Rocket_Flight_DM::Rocket_Flight_DM(const Rocket_Flight_DM &other)
    :   MATRIX_INIT(TBD, 3, 3),
        MATRIX_INIT(TBI, 3, 3),
        MATRIX_INIT(TBID, 3, 3),
        MATRIX_INIT(WEII_skew, 3, 3),
        MATRIX_INIT(TDI, 3, 3),
        MATRIX_INIT(TDE, 3, 3),
        MATRIX_INIT(TGI, 3, 3),
        VECTOR_INIT(TBI_Q, 4),
        VECTOR_INIT(TBID_Q, 4),
        VECTOR_INIT(TBDQ, 4),
        VECTOR_INIT(VBAB, 3),
        VECTOR_INIT(SBII, 3),
        VECTOR_INIT(VBII, 3),
        VECTOR_INIT(SBIIP, 3),
        VECTOR_INIT(VBIIP, 3),
        VECTOR_INIT(ABII, 3),
        VECTOR_INIT(FSPB, 3),
        VECTOR_INIT(ABIB, 3),
        VECTOR_INIT(SBEE, 3),
        VECTOR_INIT(VBEE, 3),
        VECTOR_INIT(ABEE, 3),
        VECTOR_INIT(JBII, 3),
        VECTOR_INIT(JBEE, 3),
        VECTOR_INIT(CONING, 3),
        VECTOR_INIT(NEXT_ACC, 3),
        VECTOR_INIT(VBED, 3),
        VECTOR_INIT(VBII_old, 3),
        VECTOR_INIT(WEII, 3),
        VECTOR_INIT(WBII, 3),
        VECTOR_INIT(WBIB, 3),
        VECTOR_INIT(WBIBD, 3),
        VECTOR_INIT(WBEB, 3),
        MATRIX_INIT(TLI, 3, 3),
        VECTOR_INIT(LT_euler, 3),
        VECTOR_INIT(TBLQ, 4) {
        }

Rocket_Flight_DM& Rocket_Flight_DM::operator=(const Rocket_Flight_DM& other) {
    if (&other == this)
        return *this;

    return *this;
        }

void Rocket_Flight_DM::initialize() {
    arma::mat33 TEI = grab_TEI();  // cad::tei(get_elapsed_time());
    arma::vec3 xcg = grab_xcg_0();
    build_WEII();
    arma::vec3 rhoC_1;
    rhoC_1(0) = -xcg(0) - (-8.436);
    rhoC_1(1) = 0.0;
    rhoC_1(2) = 0.0;
    // arma::mat33 TBI = kinematics->get_TBI();

    // converting geodetic lonx, latx, alt to SBII
    SBII = cad::in_geo84(lonx * RAD, latx * RAD, alt, TEI);

    // building inertial velocity
    TDI = cad::tdi84(lonx * RAD, latx * RAD, alt, TEI);
    TGI = cad::tgi84(lonx * RAD, latx * RAD, alt, TEI);

    TBD = build_psi_tht_phi_TM(psibdx * RAD, thtbdx * RAD, phibdx * RAD);
    psibd = psibdx * RAD;
    thtbd = thtbdx * RAD;
    phibd = phibdx * RAD;
    // arma::mat current_TDI = cad::tdi84(lonx * RAD, latx * RAD, alt, TEI);
    TBI = TBD * TDI;
    TLI = TBI;

    this->WBIB = this->WBEB + TBI * this->WEII;

    this->TBI_Q = Matrix2Quaternion(this->TBI);  // Convert Direct Cosine Matrix to Quaternion

    arma::mat VBEB = this->build_VBEB(alphax, betax, _dvbe);
    // arma::mat33 TBD = kinematics->get_TBD();
    // Geodetic velocity
    arma::mat VBED = trans(TBD) * VBEB;

    VBII = trans(TDI) * VBED + trans(TEI) * (WEII_skew * (TEI * SBII));
    SBIIP = SBII - trans(TBI) * rhoC_1;
    VBIIP = VBII - trans(TBI) * cross(WBIB, rhoC_1);
    arma::vec3 GRAVG = grab_GRAVG();
    this->ABII = trans(TEI) * (WEII_skew * WEII_skew * (TEI * SBII));
    FSPB = TBI * (-GRAVG + ABII);  // FSPB: body force include gravity acc
    SBEE = TEI * SBII;  // Calculate position in ECEF
    // liftoff = 0;
    NEXT_ACC = trans(TEI) * (cross(WEII, cross(WEII, (TEI * (SBIIP)))));
    Interpolation_Extrapolation_flag = 4;
}

void Rocket_Flight_DM::load_angle(double yaw, double roll, double pitch) {
    this->psibdx = yaw;
    this->phibdx = roll;
    this->thtbdx = pitch;

    // double lonx = newton->get_lonx();
    // double latx = newton->get_latx();
    // double alt  = newton->get_alt();
    // arma::mat33 TEI = grab_TEI();

    // TBD = build_psi_tht_phi_TM(psibdx * RAD, thtbdx * RAD, phibdx * RAD);

    // arma::mat current_TDI = cad::tdi84(lonx * RAD, latx * RAD, alt, TEI);
    // TBI = TBD * current_TDI;

    // this->TBI_Q = Matrix2Quaternion(this->TBI);  // Convert Direct Cosine Matrix to Quaternion
}

void Rocket_Flight_DM::load_angular_velocity(double ppx, double qqx, double rrx) {
    // arma::mat33 TBI = kinematics->get_TBI();

    // body rate wrt Earth frame in body coordinates
    this->WBEB = {ppx * RAD, qqx * RAD, rrx * RAD};
}

void Rocket_Flight_DM::load_location(double lonx, double latx, double alt) {
    this->lonx = lonx;
    this->latx = latx;
    this->alt = alt;
    // arma::mat33 TEI = grab_TEI();
    // // converting geodetic lonx, latx, alt to SBII
    // SBII = cad::in_geo84(lonx * RAD, latx * RAD, alt, TEI);

    // // building inertial velocity
    // TDI = cad::tdi84(lonx * RAD, latx * RAD, alt, TEI);
    // TGI = cad::tgi84(lonx * RAD, latx * RAD, alt, TEI);
}

void Rocket_Flight_DM::load_coning_var(double ang, double w) {
    this->con_ang = ang;
    this->con_w = w;
}

void Rocket_Flight_DM::load_geodetic_velocity(double alpha0x, double beta0x, double dvbe) {
    this->_dvbe = dvbe;
    this->alphax = alpha0x;
    this->betax = beta0x;
    // arma::mat33 TEI = grab_TEI();
    // building geodetic velocity VBED(3x1) from  alpha, beta, and dvbe
    // arma::mat VBEB = this->build_VBEB(alpha0x, beta0x, dvbe);
    // // arma::mat33 TBD = kinematics->get_TBD();
    // // Geodetic velocity
    // arma::mat VBED = trans(TBD) * VBEB;

    // VBII = trans(TDI) * VBED + trans(TEI) * (WEII_skew * (TEI * SBII));
}

void Rocket_Flight_DM::set_liftoff(unsigned int in) { liftoff = in;}

void Rocket_Flight_DM::build_WEII() {
    WEII_skew(0, 1) = -WEII3;
    WEII_skew(1, 0) =  WEII3;

    WEII(2) = WEII3;
}

arma::vec Rocket_Flight_DM::build_VBEB(double _alpha0x, double _beta0x, double _dvbe) {
    double salp = sin(_alpha0x * RAD);
    double calp = cos(_alpha0x * RAD);
    double sbet = sin(_beta0x * RAD);
    double cbet = cos(_beta0x * RAD);
    double vbeb1 = calp * cbet * _dvbe;
    double vbeb2 = sbet * _dvbe;
    double vbeb3 = salp * cbet * _dvbe;
    arma::vec3 VBEB = {vbeb1, vbeb2, vbeb3};
    return VBEB;
}

void Rocket_Flight_DM::set_reference_point(double rp) {
    reference_point = rp;
}

void Rocket_Flight_DM::propagate(double int_step) {
    double dvba = grab_dvba();
    double vmass = grab_vmass();

    arma::vec VAED = grab_VAED();
    arma::vec3 FAPB = grab_FAPB();
    arma::mat33 TEI = grab_TEI();
    arma::vec3 GRAVG = grab_GRAVG();
    // this->FSPB = calculate_fspb(FAPB, vmass);
    vibration(int_step);  // Calculate vibration angular rate
    // propagate_position_speed_acceleration(int_step);

    // propagate_WBIB(int_step, FMB, IBBB);
    /* Propagate Quaternion */
    // propagate_TBI_Q(int_step, WBIB);
    RK4(GRAVG, TEI, int_step);

    this->TBD = calculate_TBD(lonx, latx, alt);
    aux_calulate(TEI, TBI);

    if (liftoff == 1) {
        propagate_aeroloss(int_step);
        propagate_gravityloss(int_step);
        propagate_control_loss(int_step);
        VBAB = TBD * (TDE * VBEE - VAED);
        // VBAB = TBD * (VBED - VAED);
        this->alphax = calculate_alphax(VBAB);
        this->betax  = calculate_betax(VBAB, norm(VBAB));

        this->alppx = calculate_alppx(VBAB, norm(VBAB));
        this->phipx = calculate_phipx(VBAB);
    }
    // *incidence angles using wind vector VAED in geodetic coord
    // *diagnostic: calculating the inertial incidence angles
    arma::vec3 VBIB = TBI * VBII;
    this->alphaix = calculate_alphaix(VBIB);
    this->betaix  = calculate_betaix(VBIB);

    this->TBDQ = Matrix2Quaternion(this->TBD);
    Quaternion2Euler(TBDQ, Roll, Pitch, Yaw);
}

void Rocket_Flight_DM::vibration(double int_step) {
    CONING(0) = -con_ang * RAD * con_w * sin(con_w * t) * cos(con_ang * RAD);
    CONING(1) = con_ang * RAD * con_w * cos(con_w * t) * cos(con_ang * RAD);
    CONING(2) = -sin(con_ang * RAD) * con_ang * RAD * con_w;

    t += int_step;
}

void Rocket_Flight_DM::propagate_TBI(double int_step, arma::vec3 WBIB) {
    // *integrating direction cosine matrix
    INTEGRATE_MAT(TBI, trans(skew_sym(WBIB)) * this->TBI);

    // orthonormalizing TBI
    arma::mat EE = arma::eye(3, 3) - TBI * trans(TBI);
    this->TBI = TBI + EE * TBI * 0.5;

    // TBI orthogonality check
    double e1 = EE(0, 0);
    double e2 = EE(1, 1);
    double e3 = EE(2, 2);
    this->ortho_error = sqrt(e1 * e1 + e2 * e2 + e3 * e3);
}

void Rocket_Flight_DM::propagate_TBI_Q(double int_step, arma::vec3 WBIB) {
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

    this->TBI = Quaternion2Matrix(this->TBI_Q);  // Convert Quaternion to Matrix

    // TBI orthogonality check
    arma::mat TIB = trans(TBI);
    arma::mat UBI = TIB * TBI;
    double e1 = UBI(0, 0) - 1.;
    double e2 = UBI(1, 1) - 1.;
    double e3 = UBI(2, 2) - 1.;
    this->ortho_error = sqrt(e1 * e1 + e2 * e2 + e3 * e3);
}

void Rocket_Flight_DM::propagate_position_speed_acceleration(double int_step) {
    arma::vec3 GRAVG = grab_GRAVG();
    arma::mat33 TEI = grab_TEI();  // cad::tei(get_elapsed_time());

    /* Prograte S, V, A status */
    NEXT_ACC = trans(TBI) * FSPB + GRAVG;  // Strapdown Analytics 4.3-11
    /* To check wether the rocket liftoff */
    if (liftoff == 0) {
        if (norm(FSPB) - norm(GRAVG) > 0) {
            liftoff = 1;
        }
        NEXT_ACC = trans(TEI) * (cross(WEII, cross(WEII, (TEI * SBII))));
        FSPB = TBI * (-GRAVG + NEXT_ACC);  // Strapdown Analytics 4.3-14
    } else {
        FSPB = FSPB;  //+ TBI * GRAVG;  // FSPB: body force include gravity acc
    }

    arma::mat NEXT_VEL = integrate(NEXT_ACC, ABII, VBII, int_step);
    SBII = integrate(VBII, VBII_old, SBII, int_step);
    JBII = (NEXT_ACC - ABII)/int_step;  // Calculate Jerk in ECI
    ABII = NEXT_ACC;
    VBII_old = VBII;
    VBII = NEXT_VEL;
}
arma::mat Rocket_Flight_DM::calculate_TBD(double lonx, double latx, double alt) {
    // _Euler_ angles
    arma::mat33 TEI = grab_TEI();

    arma::mat TDI = cad::tdi84(lonx * RAD, latx * RAD, alt, TEI);
    return this->TBI * trans(TDI);
}

double Rocket_Flight_DM::calculate_alphaix(arma::vec3 VBIB) {
    return atan2(VBIB(2), VBIB(0)) * DEG;
}

void Rocket_Flight_DM::propagate_aeroloss(double int_step) {
    arma::vec3 FAPB = grab_FAPB();
    double vmass = grab_vmass();
    // calculate aero loss`:`
    FAPB = FAPB * (1. / vmass);
    _aero_loss = _aero_loss + norm(FAPB) * int_step;
}

void Rocket_Flight_DM::propagate_control_loss(double int_step) {
    arma::vec6 Q_TVC = grab_Q_TVC();
    double vmass = grab_vmass();
    arma::vec3 A_TVC_BODY;
    A_TVC_BODY(0) = Q_TVC(0) / vmass;
    A_TVC_BODY(1) = Q_TVC(1) / vmass;
    A_TVC_BODY(2) = Q_TVC(2) / vmass;

    A_TVC_BODY = TBI * A_TVC_BODY;

    control_loss = control_loss + (fabs(A_TVC_BODY(1)) + fabs(A_TVC_BODY(2))) * int_step;
}

void Rocket_Flight_DM::propagate_gravityloss(double int_step) {
    // calculate gravity loss
    double grav = grab_grav();
    gravity_loss = gravity_loss + grav * sin(get_thtvdx() * RAD) * int_step;
}

double Rocket_Flight_DM::calculate_betaix(arma::vec3 VBIB) {
    double dvbi = norm(VBIB);
    return asin(VBIB(1) / dvbi) * DEG;
}

arma::vec3 Rocket_Flight_DM::calculate_fspb(arma::vec3 FAPB, double vmass) {
    /* Stored Value due to coherence with other models */
    return FAPB * (1. / vmass);
}

double Rocket_Flight_DM::calculate_alppx(arma::vec3 VBAB, double dvba) {
    // incidence angles in load factor plane (aeroballistic)
    double dum = VBAB(0) / dvba;

    if (fabs(dum) > 1)
        dum = 1 * sign(dum);
    double alpp = acos(dum);

    return alpp * DEG;
}

double Rocket_Flight_DM::calculate_phipx(arma::vec3 VBAB) {
    double phip = 0;
    // Changed according to comments, not original code, refer commit:b613a992
    if (VBAB(1) == 0 && VBAB(2) == 0) {
        phip = 0.;
    } else if (fabs(VBAB(1)) < arma::datum::eps) {
        // note: if vbeb2 is <EPS the value phip is forced to be 0 or PI
        //      to prevent oscillations
        if (VBAB(2) > 0) phip = 0;
        if (VBAB(2) < 0) phip = PI;
    } else {
        phip = atan2(VBAB(1), VBAB(2));
    }

    return phip * DEG;
}

double Rocket_Flight_DM::calculate_alphax(arma::vec3 VBAB) {
    double alpha = atan2(VBAB(2), VBAB(0));
    return alpha * DEG;
}

double Rocket_Flight_DM::calculate_betax(arma::vec3 VBAB, double dvba) {
    double beta = asin(VBAB(1) / dvba);
    return beta * DEG;
}

void Rocket_Flight_DM::propagate_WBIB(double int_step, arma::vec3 FMB, arma::mat33 IBBB) {
    // integrating the angular velocity acc wrt the inertial frame in body coord
    // Using Armadillo solve for higher accuracy, otherwise will faile the 1ppm test
    INTEGRATE_MAT(WBIB, arma::solve(IBBB, (FMB - skew_sym(this->WBIB) * IBBB * this->WBIB)));
}

arma::vec3 Rocket_Flight_DM::calculate_WBII(arma::mat33 TBI) {
    return trans(TBI) * this->WBIB;
}

arma::vec3 Rocket_Flight_DM::calculate_WBEB(arma::mat33 TBI) {
    return this->WBIB - TBI * this->WEII;
}

double Rocket_Flight_DM::get_psibdx() {
    double cthtbd = 0;

    get_thtbdx_in(cthtbd);

    // yaw angle: 'psibd'
    double cpsi = TBD(0, 0) / cthtbd;
    if (fabs(cpsi) > 1) {
        cpsi = 1 * sign(cpsi);
    }
    psibd = acos(cpsi) * sign(TBD(0, 1));
    // psibd = atan2( 2. * (TBDQ(1) * TBDQ(2) + TBDQ(0) * TBDQ(3) ), (1. - 2. * (TBDQ(2) * TBDQ(2) + TBDQ(3) * TBDQ(3))) );

    return DEG * psibd;
}

double Rocket_Flight_DM::get_thtbdx() {
    double cthtbd = 0;
    return get_thtbdx_in(cthtbd);
}

double Rocket_Flight_DM::get_thtbdx_in(double &cthtbd) {
    // *geodetic _Euler_ angles
    // pitch angle: 'thtbd'
    // note: when |tbd13| >= 1, thtbd = +- pi/2, but cos(thtbd) is
    //      forced to be a small positive number to prevent division by zero
    if (fabs(TBD(0, 2)) < 1) {
        thtbd = asin(-TBD(0, 2));
        cthtbd = cos(thtbd);
    } else {
        thtbd = PI / 2 * sign(-TBD(0, 2));
        cthtbd = arma::datum::eps;
    }
    // thtbd = asin(  2 * (TBDQ(0) * TBDQ(2) - TBDQ(1) * TBDQ(3)));

    return DEG * thtbd;
}

double Rocket_Flight_DM::get_phibdx() {
    double cthtbd = 0;

    get_thtbdx_in(cthtbd);

    // roll angle: 'phibdc'
    double cphi = TBD(2, 2) / cthtbd;
    if (fabs(cphi) > 1) {
        cphi = 1 * sign(cphi);
    }
    phibd = acos(cphi) * sign(TBD(1, 2));
    // phibd = atan2( 2. * (TBDQ(2) * TBDQ(3) + TBDQ(0) * TBDQ(1) ), (1. - 2. * (TBDQ(2) * TBDQ(2) + TBDQ(1) * TBDQ(1))));


    return DEG * phibd;
}

void Rocket_Flight_DM::update_diagnostic_attributes(double int_step) {
    this->thtbdx = get_thtbdx();
    this->psibdx = get_psibdx();
    this->phibdx = get_phibdx();

    this->ppx = get_ppx();
    this->qqx = get_qqx();
    this->rrx = get_rrx();

    _dbi = get_dbi();
    _dvbi = get_dvbi();

    _dvbe = get_dvbe();
    _psivdx = get_psivdx();
    _thtvdx = get_thtvdx();

    double vbed1 = get_VBED()[0];
    double vbed2 = get_VBED()[1];
    _grndtrck += sqrt(vbed1 * vbed1 + vbed2 * vbed2) * int_step * REARTH / get_dbi();
    _gndtrkmx = 0.001 * _grndtrck;
    _gndtrnmx = NMILES * _grndtrck;

    _ayx =  FSPB(1) / AGRAV;
    _anx = -FSPB(2) / AGRAV;

    if (liftoff == 1) {
        // T.M. of geographic velocity wrt geodetic coordinates
        arma::mat TVD(&_TVD[0][0], 3, 3, false, true);
        TVD = build_psivg_thtvg_TM(_psivdx * RAD, _thtvdx * RAD);
        orbital(SBII, VBII, get_dbi());
    }
}

void Rocket_Flight_DM::orbital(arma::vec3 SBII, arma::vec3 VBII, double dbi) {
    // calculate orbital elements
    int cadorbin_flag = cad::orb_in(_semi_major, _eccentricity, _inclination, _lon_anodex, _arg_perix, _true_anomx, SBII, VBII);
    _ha = (1. + _eccentricity) * _semi_major - REARTH;
    _hp = (1. - _eccentricity) * _semi_major - REARTH;
    _ref_alt = dbi - REARTH;
}

void Rocket_Flight_DM::aux_calulate(arma::mat33 TEI, arma::mat33 TBI) {
    double lon, lat, al;
    arma::mat33 TBL;

    // angular velocity wrt inertial frame in inertial coordinates
    this->WBII = calculate_WBII(TBI);

    // angular velocity wrt Earth in body coordinates
    this->WBEB = calculate_WBEB(TBI);

    SBEE_old = SBEE;
    VBEE_old = VBEE;
    ABEE_old = ABEE;

    ABIB = TBI * ABII;
    SBEE = TEI * SBII;  // Calculate position in ECEF
    VBEE = TEI * VBII - cross(WEII, SBEE);  // Calculate velocity in ECEF
    ABEE = TEI * ABII - cross(WEII, VBEE) - cross(WEII, VBEE) - cross(WEII, cross(WEII, SBEE));
    JBEE = TEI * JBII - cross(WEII, cross(WEII, cross(WEII, SBEE))) - cross(WEII, cross(WEII, VBEE)) - cross(WEII, ABEE);

    // Calculate lon lat alt
    std::tie(lon, lat, al) = cad::geo84_in(SBII, TEI);
    this->lonx = lon * DEG;
    this->latx = lat * DEG;
    this->alt  = al;
    // std::cout<<alt<<std::endl;
    if (liftoff == 1) assert(alt >= 0.0 && " *** Stop: Ground impact detected !! *** ");

    TDI = cad::tdi84(lon, lat, al, TEI);
    TDE = cad::tde84(lon, lat, al);
    TGI = cad::tgi84(lon, lat, al, TEI);
    TBL = TBI * trans(TLI);
    LT_euler = euler_angle(TBL);

    VBED = TDE * VBEE;
}

void Rocket_Flight_DM::RK4F(arma::vec3 GRAVG, arma::mat33 TEI, double int_step,
                            arma::vec3 &K1, arma::vec3 &K2, arma::vec3 &K3, arma::vec4 &K4,
                            double &K5, double &K6, double &K7, double &K8) {
    double vmass = grab_vmass();
    double thrust = grab_thrust();
    // arma::vec3 FMB = grab_FMB();
    // arma::vec3 FAPB = grab_FAPB();
    // arma::mat33 IBBB = grab_IBBB();
    arma::vec4 TBID_Q_NEW;
    arma::vec3 rhoC_IMU;

    collect_forces_and_propagate();

    arma::vec3 ddrP_1 = grab_ddrP_1();
    arma::vec3 xcg0 = grab_xcg_0();
    arma::vec3 WBIBD_new = grab_ddang_1();
    double ddang_slosh_theta = grab_ddang_slosh_theta();
    double ddang_slosh_psi = grab_ddang_slosh_psi();

    rhoC_IMU(0) = -xcg0(0) - (reference_point);
    rhoC_IMU(1) = 0.0;
    rhoC_IMU(2) = 0.0;
    arma::vec3 ddrhoC_IMU = cross(WBIBD, rhoC_IMU) + cross(WBIB, cross(WBIB, rhoC_IMU));
    NEXT_ACC = ddrP_1;

    /******************************************************************************************/
    /* Prograte S, V, A status */
    // arma::vec3 WBIBD_new = arma::solve(IBBB, (FMB - skew_sym(this->WBIB) * IBBB * this->WBIB));
    // NEXT_ACC = trans(TBI) * FSPB + GRAVG;  // Strapdown Analytics 4.3-11
    // if (liftoff == 0) {
    //     if (norm(FSPB) - norm(GRAVG) > 0) {
    //         liftoff = 1;
    //     }
    //     NEXT_ACC = trans(TEI) * (cross(WEII, cross(WEII, (TEI * SBII))));
    //     FSPB = TBI * (-GRAVG + NEXT_ACC);  // Strapdown Analytics 4.3-14
    // } else {
    //     FSPB = FSPB;  //+ TBI * GRAVG;  // FSPB: body force include gravity acc
    // }
    /******************************************************************************************/
    if (liftoff == 0) {
        if ((thrust - norm(vmass * GRAVG)) > 0) {
            liftoff = 1;
        }
        NEXT_ACC = trans(TEI) * (cross(WEII, cross(WEII, (TEI * (SBIIP)))));
        FSPB = TBI * (-GRAVG + NEXT_ACC + ddrhoC_IMU);  // Strapdown Analytics 4.3-14
    } else {
        FSPB = TBI * (NEXT_ACC - GRAVG + ddrhoC_IMU);  //+ TBI * GRAVG;  // FSPB: body force include gravity acc
    }
    /* Prepare for orthonormalization */
    double quat_metric = TBI_Q(0) * TBI_Q(0) + TBI_Q(1) * TBI_Q(1) + TBI_Q(2) * TBI_Q(2) + TBI_Q(3) * TBI_Q(3);
    double erq = 1. - quat_metric;

    /* Calculate Previous states */  //  Zipfel p.141
    TBID_Q_NEW(0) = 0.5 * (-WBIB(0) * TBI_Q(1) - WBIB(1) * TBI_Q(2) - WBIB(2) * TBI_Q(3)) + 50. * erq * TBI_Q(0);
    TBID_Q_NEW(1) = 0.5 * (WBIB(0) * TBI_Q(0) + WBIB(2) * TBI_Q(2) - WBIB(1) * TBI_Q(3)) + 50. * erq * TBI_Q(1);
    TBID_Q_NEW(2) = 0.5 * (WBIB(1) * TBI_Q(0) - WBIB(2) * TBI_Q(1) + WBIB(0) * TBI_Q(3)) + 50. * erq * TBI_Q(2);
    TBID_Q_NEW(3) = 0.5 * (WBIB(2) * TBI_Q(0) + WBIB(1) * TBI_Q(1) - WBIB(0) * TBI_Q(2)) + 50. * erq * TBI_Q(3);

    K1 = NEXT_ACC;
    K2 = VBII;
    K3 = WBIBD_new;
    K4 = TBID_Q_NEW;
    K5 = ddang_slosh_theta;
    K6 = dang_slosh_theta;
    K7 = ddang_slosh_psi;
    K8 = dang_slosh_psi;
    ABII = NEXT_ACC;
}
void Rocket_Flight_DM::RK4(arma::vec3 GRAVG, arma::mat33 TEI, double int_step) {
    arma::vec3 K11, K21, K31, K41;
    arma::vec3 K12, K22, K32, K42;
    arma::vec3 K13, K23, K33, K43;
    arma::vec4 K14, K24, K34, K44;
    double K15, K25, K35, K45;
    double K16, K26, K36, K46;
    double K17, K27, K37, K47;
    double K18, K28, K38, K48;
    arma::vec4 TBI_Q_post;
    arma::vec3 VBIIP_post, SBIIP_post, WBIB_post;
    double dang_slosh_theta_post, ang_slosh_theta_post, dang_slosh_psi_post, ang_slosh_psi_post;

    arma::vec3 ddang_1 = grab_ddang_1();
    arma::vec3 rhoC_1;
    arma::vec3 xcg0 = grab_xcg_0();
    rhoC_1(0) = -xcg0(0) - (reference_point);
    rhoC_1(1) = 0.0;
    rhoC_1(2) = 0.0;

    SBIIP_post = SBIIP;  // - trans(TBI) * rhoC_1;
    VBIIP_post = VBIIP;  // - trans(TBI) * cross(WBIB, rhoC_1);
    WBIB_post = WBIB;
    TBI_Q_post = TBI_Q;
    dang_slosh_theta_post = dang_slosh_theta;
    ang_slosh_theta_post = ang_slosh_theta;
    dang_slosh_psi_post = dang_slosh_psi;
    ang_slosh_psi_post = ang_slosh_psi;

    RK4F(GRAVG, TEI, int_step, K11, K12, K13, K14, K15, K16, K17, K18);
    VBIIP = VBIIP_post + K11 * 0.5 * int_step;
    SBIIP = SBIIP_post + K12 * 0.5 * int_step;
    WBIB = WBIB_post + K13 * 0.5 * int_step;
    TBI_Q = TBI_Q_post + K14 * 0.5 *int_step;
    ang_slosh_theta = ang_slosh_theta_post + K16 * 0.5 * int_step;
    ang_slosh_psi = ang_slosh_psi_post + K18 * 0.5 * int_step;
    dang_slosh_theta = dang_slosh_theta_post + K15 * 0.5 * int_step;
    dang_slosh_psi = dang_slosh_psi_post + K17 * 0.5 * int_step;
    this->TBI = Quaternion2Matrix(this->TBI_Q);  // Convert Quaternion to Matrix

    RK4F(GRAVG, TEI, int_step, K21, K22, K23, K24, K25, K26, K27, K28);
    VBIIP = VBIIP_post + K21 * 0.5 * int_step;
    SBIIP = SBIIP_post + K22 * 0.5 * int_step;
    WBIB = WBIB_post + K23 * 0.5 * int_step;
    TBI_Q = TBI_Q_post + K24 * 0.5 *int_step;
    ang_slosh_theta = ang_slosh_theta_post + K26 * 0.5 * int_step;
    ang_slosh_psi = ang_slosh_psi_post + K28 * 0.5 * int_step;
    dang_slosh_theta = dang_slosh_theta_post + K25 * 0.5 * int_step;
    dang_slosh_psi = dang_slosh_psi_post + K27 * 0.5 * int_step;
    this->TBI = Quaternion2Matrix(this->TBI_Q);  // Convert Quaternion to Matrix

    RK4F(GRAVG, TEI, int_step, K31, K32, K33, K34, K35, K36, K37, K38);
    VBIIP = VBIIP_post + K31 * int_step;
    SBIIP = SBIIP_post + K32 * int_step;
    WBIB = WBIB_post + K33 * int_step;
    TBI_Q = TBI_Q_post + K34 *int_step;
    ang_slosh_theta = ang_slosh_theta_post + K36 * int_step;
    ang_slosh_psi = ang_slosh_psi_post + K38 * int_step;
    dang_slosh_theta = dang_slosh_theta_post + K35 * int_step;
    dang_slosh_psi = dang_slosh_psi_post + K37 * int_step;
    this->TBI = Quaternion2Matrix(this->TBI_Q);  // Convert Quaternion to Matrix

    RK4F(GRAVG, TEI, int_step, K41, K42, K43, K44, K45, K46, K47, K48);
    VBIIP = VBIIP_post + (int_step / 6.0) * (K11 + 2.0 * K21 + 2.0 * K31 + K41);
    SBIIP = SBIIP_post + (int_step / 6.0) * (K12 + 2.0 * K22 + 2.0 * K32 + K42);
    WBIB = WBIB_post + (int_step / 6.0) * (K13 + 2.0 * K23 + 2.0 * K33 + K43);
    TBI_Q = TBI_Q_post + (int_step / 6.0) * (K14 + 2.0 * K24 + 2.0 * K34 + K44);
    ang_slosh_theta = ang_slosh_theta_post + (int_step / 6.0) * (K16 + 2.0 * K26 + 2.0 * K36 + K46);
    ang_slosh_psi = ang_slosh_psi_post + (int_step / 6.0) * (K18 + 2.0 * K28 + 2.0 * K38 + K48);
    dang_slosh_theta = dang_slosh_theta_post + (int_step / 6.0) * (K15 + 2.0 * K25 + 2.0 * K35 + K45);
    dang_slosh_psi = dang_slosh_psi_post + (int_step / 6.0) * (K17 + 2.0 * K27 + 2.0 * K37 + K47);
    this->TBI = Quaternion2Matrix(this->TBI_Q);  // Convert Quaternion to Matrix

    WBIBD = ddang_1;
    SBII = SBIIP + trans(TBI) * rhoC_1;
    VBII = VBIIP + trans(TBI) * cross(WBIB, rhoC_1);
    /********************************************************************************/
    // arma::vec3 VBII_post, SBII_post;

    // SBII_post = SBII;// - trans(TBI) * rhoC_1;
    // VBII_post = VBII;// - trans(TBI) * cross(WBIB, rhoC_1);
    // WBIB_post = WBIB;
    // TBI_Q_post = TBI_Q;

    // RK4F(GRAVG, TEI, int_step, K11, K12, K13, K14);
    // VBII = VBII_post + K11 * 0.5 * int_step;
    // SBII = SBII_post + K12 * 0.5 * int_step;
    // WBIB = WBIB_post + K13 * 0.5 * int_step;
    // TBI_Q = TBI_Q_post + K14 * 0.5 *int_step;
    // this->TBI = Quaternion2Matrix(this->TBI_Q);  // Convert Quaternion to Matrix

    // RK4F(GRAVG, TEI, int_step, K21, K22, K23, K24);
    // VBII = VBII_post + K21 * 0.5 * int_step;
    // SBII = SBII_post + K22 * 0.5 * int_step;
    // WBIB = WBIB_post + K23 * 0.5 * int_step;
    // TBI_Q = TBI_Q_post + K24 * 0.5 *int_step;
    // this->TBI = Quaternion2Matrix(this->TBI_Q);  // Convert Quaternion to Matrix

    // RK4F(GRAVG, TEI, int_step, K31, K32, K33, K34);
    // VBII = VBII_post + K31 * int_step;
    // SBII = SBII_post + K32 * int_step;
    // WBIB = WBIB_post + K33 * int_step;
    // TBI_Q = TBI_Q_post + K34 *int_step;
    // this->TBI = Quaternion2Matrix(this->TBI_Q);  // Convert Quaternion to Matrix

    // RK4F(GRAVG, TEI, int_step, K41, K42, K43, K44);
    // VBII = VBII_post + (int_step / 6.0) * (K11 + 2.0 * K21 + 2.0 * K31 + K41);
    // SBII = SBII_post + (int_step / 6.0) * (K12 + 2.0 * K22 + 2.0 * K32 + K42);
    // WBIB = WBIB_post + (int_step / 6.0) * (K13 + 2.0 * K23 + 2.0 * K33 + K43);
    // TBI_Q = TBI_Q_post + (int_step / 6.0) * (K14 + 2.0 * K24 + 2.0 * K34 + K44);
    // this->TBI = Quaternion2Matrix(this->TBI_Q);  // Convert Quaternion to Matrix
}
double Rocket_Flight_DM::get_alppx() { return alppx; }
double Rocket_Flight_DM::get_phipx() { return phipx; }
double Rocket_Flight_DM::get_alphax() { return alphax; }
double Rocket_Flight_DM::get_betax() { return betax; }
double Rocket_Flight_DM::get_ppx() { return this->WBEB(0) * DEG; }
double Rocket_Flight_DM::get_qqx() { return this->WBEB(1) * DEG; }
double Rocket_Flight_DM::get_rrx() { return this->WBEB(2) * DEG; }
double Rocket_Flight_DM::get_alt() { return alt; }
double Rocket_Flight_DM::get_lonx() { return lonx; }
double Rocket_Flight_DM::get_latx() { return latx; }
double Rocket_Flight_DM::get_dbi() { return norm(SBII); }
double Rocket_Flight_DM::get_dvbi() { return norm(VBII); }
double Rocket_Flight_DM::get_dvbe() { return pol_from_cart(get_VBED())(0); }
double Rocket_Flight_DM::get_thtvdx() { return DEG * pol_from_cart(get_VBED())(2); }
double Rocket_Flight_DM::get_psivdx() { return DEG * pol_from_cart(get_VBED())(1); }

arma::mat Rocket_Flight_DM::get_TGI() { return TGI; }
arma::mat Rocket_Flight_DM::get_TDE() { return TDE; }
arma::mat Rocket_Flight_DM::get_TBD() { return TBD; }
arma::mat Rocket_Flight_DM::get_TBI() { return TBI; }
arma::vec3 Rocket_Flight_DM::get_VBAB() { return VBAB; }
arma::vec3 Rocket_Flight_DM::get_WBII() { return this->WBII; }
arma::vec3 Rocket_Flight_DM::get_WBIB() { return this->WBIB; }
arma::vec3 Rocket_Flight_DM::get_WBIBD() { return this->WBIBD; }
arma::vec3 Rocket_Flight_DM::get_WEII() { return this->WEII; }
arma::vec3 Rocket_Flight_DM::get_CONING() { return CONING;}
arma::vec3 Rocket_Flight_DM::get_VBED() { return TDI * (VBII - WEII_skew * SBII); }
arma::vec3 Rocket_Flight_DM::get_VBII() { return VBII; }
arma::vec3 Rocket_Flight_DM::get_ABII() { return ABII; }
arma::vec3 Rocket_Flight_DM::get_SBII() { return SBII; }
arma::vec3 Rocket_Flight_DM::get_FSPB() { return FSPB; }
arma::vec3 Rocket_Flight_DM::get_SBEE() { return SBEE; }
arma::vec3 Rocket_Flight_DM::get_VBEE() { return VBEE; }
arma::vec3 Rocket_Flight_DM::get_SBEE_test() { return SBEE_test; }
arma::vec3 Rocket_Flight_DM::get_VBEE_test() { return VBEE_test; }
arma::vec3 Rocket_Flight_DM::get_ABEE_test() { return ABEE_test; }
arma::vec3 Rocket_Flight_DM::get_NEXT_ACC() { return NEXT_ACC; }
double Rocket_Flight_DM::get_dang_slosh_theta() { return dang_slosh_theta; }
double Rocket_Flight_DM::get_ang_slosh_theta() { return ang_slosh_theta; }
double Rocket_Flight_DM::get_dang_slosh_psi() { return dang_slosh_psi; }
double Rocket_Flight_DM::get_ang_slosh_psi() { return ang_slosh_psi; }

double *Rocket_Flight_DM::get_double_SBEE() { return _SBEE; }
double *Rocket_Flight_DM::get_double_VBEE() { return _VBEE; }
double *Rocket_Flight_DM::get_double_ABEE() { return _ABEE; }
double *Rocket_Flight_DM::get_double_JBEE() { return _JBEE; }
double *Rocket_Flight_DM::get_double_WBEB() { return _WBEB; }
double *Rocket_Flight_DM::get_double_SBEE_test() { return _SBEE_test; }
double *Rocket_Flight_DM::get_double_VBEE_test() { return _VBEE_test; }
double *Rocket_Flight_DM::get_double_ABEE_test() { return _ABEE_test; }
double Rocket_Flight_DM::get_double_psibd() { return psibd; }
double Rocket_Flight_DM::get_double_thtbd() { return thtbd; }
double Rocket_Flight_DM::get_double_phibd() { return phibd; }

unsigned int Rocket_Flight_DM::get_liftoff() { return liftoff; }

int Rocket_Flight_DM::enqueue_to_simgen_buffer(struct icf_ctrlblk_t* C, double ext_porlation) {
    struct simgen_motion_data_t motion_info;
    double (*pos)[3];
    double (*vel)[3];
    double (*accel)[3];
    pos = (ext_porlation == 0.0) ? &_SBEE : &_SBEE_test;
    vel = (ext_porlation == 0.0) ? &_VBEE : &_VBEE_test;
    accel = (ext_porlation == 0.0) ? &_ABEE : &_ABEE_test;
    motion_info.sim_time.second = exec_get_sim_time() + ext_porlation;
    motion_info.cmd_idx = REMOTE_MOTION_CMD_MOT;
    motion_info.vehicle_id = 1;
    memcpy(&motion_info.position_xyz, pos, sizeof(double) * 3);
    memcpy(&motion_info.velocity_xyz, vel, sizeof(double) * 3);
    memcpy(&motion_info.acceleration_xyz, accel, sizeof(double) * 3);
    motion_info.jerk_xyz[0] = 0.0;
    motion_info.jerk_xyz[1] = 0.0;
    motion_info.jerk_xyz[2] = 0.0;
    motion_info.heb[0] = psibd;
    motion_info.heb[1] = thtbd;
    motion_info.heb[2] = phibd;
    memcpy(&motion_info.angular_velocity, &_WBEB, sizeof(double) * 3);
    motion_info.angular_acceleration[0] = 0.0;
    motion_info.angular_acceleration[1] = 0.0;
    motion_info.angular_acceleration[2] = 0.0;
    motion_info.angular_jerk[0] = 0.0;
    motion_info.angular_jerk[1] = 0.0;
    motion_info.angular_jerk[2] = 0.0;
    icf_tx_enqueue(C, EGSE_TX_GPSRF_EMU_QIDX, &motion_info, sizeof(struct simgen_motion_data_t));
    return 0;
}

int  Rocket_Flight_DM::stand_still_motion_data(struct icf_ctrlblk_t* C, double ext_porlation) {
    struct simgen_motion_data_t motion_info;
    double (*pos)[3];
    double (*vel)[3];
    double (*accel)[3];
    pos = (ext_porlation == 0.0) ? &_SBEE : &_SBEE_test;
    vel = (ext_porlation == 0.0) ? &_VBEE : &_VBEE_test;
    accel = (ext_porlation == 0.0) ? &_ABEE : &_ABEE_test;
    motion_info.sim_time.second = exec_get_sim_time() + ext_porlation;
    motion_info.cmd_idx = REMOTE_MOTION_CMD_MOT;
    motion_info.vehicle_id = 1;
    memcpy(&motion_info.position_xyz, pos, sizeof(double) * 3);
    memcpy(&motion_info.velocity_xyz, vel, sizeof(double) * 3);
    memcpy(&motion_info.acceleration_xyz, accel, sizeof(double) * 3);
    motion_info.jerk_xyz[0] = 0.0;
    motion_info.jerk_xyz[1] = 0.0;
    motion_info.jerk_xyz[2] = 0.0;
    /* Heading, Yaw, Z axis, Down */
    motion_info.heb[0] = 1.5707963268;
    /* Elevation, Pitch, Y axis, East */
    motion_info.heb[1] = 1.5707963268;
    /* Bank, Roll, X axis, North */
    motion_info.heb[2] = 0.0;
    memcpy(&motion_info.angular_velocity, &_WBEB, sizeof(double) * 3);
    motion_info.angular_acceleration[0] = 0.0;
    motion_info.angular_acceleration[1] = 0.0;
    motion_info.angular_acceleration[2] = 0.0;
    motion_info.angular_jerk[0] = 0.0;
    motion_info.angular_jerk[1] = 0.0;
    motion_info.angular_jerk[2] = 0.0;
    icf_tx_enqueue(C, EGSE_TX_GPSRF_EMU_QIDX, &motion_info, sizeof(struct simgen_motion_data_t));
    return 0;
}

void Rocket_Flight_DM::Interpolation_Extrapolation(double T, double int_step, double ext_porlation) {
    arma::vec3 A, B, C, D, E, F;
    // double t;
    // Interpolation_Extrapolation_flag++;
    // if (Interpolation_Extrapolation_flag == 5) {
    //     Interpolation_Extrapolation_flag = 0;
    // }
    // t = Interpolation_Extrapolation_flag * int_step;

    A = SBEE_old;
    B = VBEE_old;
    // C = ABEE_old / 2.0;
    // D = (-20.0 * SBEE_old + 20.0 * SBEE - 12.0 * T * VBEE_old - 8.0 * T * VBEE - 3.0 * T * T * ABEE_old + T * T * ABEE) / (2.0 * T * T * T);
    // E = (30.0 * SBEE_old - 30.0 * SBEE + 16.0 * T * VBEE_old + 14.0 * T * VBEE + 3.0 * T * T * ABEE_old - 2.0 * T * T * ABEE) / (2.0 * T * T * T * T);
    // F = (-12.0 * SBEE_old + 12.0 * SBEE - 6.0 * T * VBEE_old - 6.0 * T * VBEE - T * T * ABEE_old + T * T * ABEE) / (2.0 * T * T * T * T * T);

    // SBEE_test = A + B * t + C * t * t + D * t * t * t
    //             + E * t * t * t * t + F * t * t * t * t * t;
    // VBEE_test = B + 2.0 * C * t + 3.0 * D * t * t + 4.0 * E * t * t * t
    //             + 5.0 * F * t * t * t * t;
    // ABEE_test = 2.0 * C + 6.0 * D * t + 12.0 * E * t * t + 20.0 * F * t * t * t;

    // C = 3.0 * (SBEE - SBEE_old) / (T * T) - (VBEE + 2.0 * VBEE_old) / T;
    // D = 2.0 * (SBEE_old - SBEE) / (T * T * T) + (VBEE + VBEE_old) / (T * T);

    // SBEE_test = A + B * t + C * t * t + D * t * t * t;
    // VBEE_test = B + 2.0 * C * t + 3.0 * D * t * t
    SBEE_test = SBEE + ((SBEE - A) / T) * ext_porlation * 1000.0 * int_step;
    VBEE_test = VBEE + ((VBEE - B) / T) * ext_porlation * 1000.0 * int_step;
}

arma::vec3 Rocket_Flight_DM::euler_angle(arma::mat33 TBD) {
    double psibdc(0), thtbdc(0), phibdc(0);
    double cthtbd(0);

    double mroll = 0;

    double tbd13 = TBD(0, 2);
    double tbd11 = TBD(0, 0);
    double tbd33 = TBD(2, 2);
    double tbd12 = TBD(0, 1);
    double tbd23 = TBD(1, 2);

    arma::vec3 euler_ang;
    // *geodetic Euler angles
    // computed pitch angle: 'thtbdc'
    // note: when |tbd13| >= 1, thtbdc = +- pi/2, but cos(thtbdc) is
    // forced to be a small positive number to prevent division by zero
    if (fabs(tbd13) < 1) {
        thtbdc = asin(-tbd13);
        cthtbd = cos(thtbdc);
    } else {
        thtbdc = PI / 2 * sign(-tbd13);
        cthtbd = EPS;
    }
    // computed yaw angle: 'psibdc'
    double cpsi = tbd11 / cthtbd;
    if (fabs(cpsi) > 1)
        cpsi = 1 * sign(cpsi);
    psibdc = acos(cpsi) * sign(tbd12);

    // computed roll angle: 'phibdc'
    double cphi = tbd33 / cthtbd;
    if (fabs(cphi) > 1)
        cphi = 1 * sign(cphi);

    // selecting the Euler roll angle of flight mechanics (not for thtbdc=90 or
    // =-90deg)
    if (mroll == 0 || mroll == 1)
        // roll feedback for right side up
        phibdc = acos(cphi) * sign(tbd23);
    else if (mroll == 2)
        // roll feedback for inverted flight
        phibdc = acos(-cphi) * sign(-tbd23);

    euler_ang(0) = phibdc;
    euler_ang(1) = thtbdc;
    euler_ang(2) = psibdc;

    return euler_ang;
}
