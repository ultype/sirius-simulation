#include "rocket/Newton.hh"
#include "sim_services/include/simtime.h"

#include "aux/utility_header.hh"
#include "aux/aux.hh"

Newton::Newton(Kinematics &kine, _Euler_ &elr, Environment &env, Propulsion &prop, Forces &forc)
    :   kinematics(&kine), euler(&elr), environment(&env), propulsion(&prop), forces(&forc),
        MATRIX_INIT(WEII, 3, 3),
        MATRIX_INIT(TDI, 3, 3),
        MATRIX_INIT(TGI, 3, 3),
        VECTOR_INIT(SBII, 3),
        VECTOR_INIT(VBII, 3),
        VECTOR_INIT(ABII, 3),
        VECTOR_INIT(FSPB, 3)
{
    this->default_data();
}

Newton::Newton(const Newton& other)
    :   kinematics(other.kinematics), euler(other.euler), environment(other.environment), propulsion(other.propulsion), forces(other.forces),
        MATRIX_INIT(WEII, 3, 3),
        MATRIX_INIT(TDI, 3, 3),
        MATRIX_INIT(TGI, 3, 3),
        VECTOR_INIT(SBII, 3),
        VECTOR_INIT(VBII, 3),
        VECTOR_INIT(ABII, 3),
        VECTOR_INIT(FSPB, 3)
{
    this->default_data();

    /* Propagative Stats */
    this->alt = other.alt;
    this->lonx = other.lonx;
    this->latx = other.latx;
    this->SBII = other.SBII;
    this->VBII = other.VBII;
    this->ABII = other.ABII;

    this->TGI = other.TGI;
    this->TDI = other.TDI;
    this->aero_loss = other.aero_loss;
    this->gravity_loss = other.gravity_loss;

    this->FSPB = other.FSPB;
}

Newton& Newton::operator=(const Newton& other){
    if(&other == this)
        return *this;

    this->kinematics = other.kinematics;
    this->environment = other.environment;
    this->euler = other.euler;
    this->propulsion = other.propulsion;
    this->forces = other.forces;

    /* Propagative Stats */
    this->alt = other.alt;
    this->lonx = other.lonx;
    this->latx = other.latx;
    this->SBII = other.SBII;
    this->VBII = other.VBII;
    this->ABII = other.ABII;

    this->TGI = other.TGI;
    this->TDI = other.TDI;
    this->aero_loss = other.aero_loss;
    this->gravity_loss = other.gravity_loss;

    this->FSPB = other.FSPB;

    return *this;
}

void Newton::default_data(){
    //Earth's angular velocity skew-symmetric matrix (3x3)
    this->WEII = this->build_WEII();

    this->SBII.zeros();
    this->VBII.zeros();
    this->ABII.zeros();

    this->FSPB.zeros();

    this->TDI.zeros();
    this->TGI.zeros();

    alt = 0;
    lonx = 0;
    latx = 0;

    aero_loss = 0;
    gravity_loss = 0;

}

void Newton::initialize(){
}

arma::mat Newton::build_WEII(){
    arma::mat33 WEII;
    WEII.zeros();
    WEII(0, 1) = -WEII3;
    WEII(1, 0) =  WEII3;
    return WEII;
}

arma::vec Newton::build_VBEB(double _alpha0x, double _beta0x, double _dvbe){
    double salp = sin(_alpha0x * RAD);
    double calp = cos(_alpha0x * RAD);
    double sbet = sin(_beta0x * RAD);
    double cbet = cos(_beta0x * RAD);
    double vbeb1 = calp * cbet * _dvbe;
    double vbeb2 = sbet * _dvbe;
    double vbeb3 = salp * cbet * _dvbe;
    arma::vec3 VBEB= {vbeb1, vbeb2, vbeb3};
    return VBEB;
}


void Newton::load_location(double lonx, double latx, double alt){
    this->lonx = lonx;
    this->latx = latx;
    this->alt = alt;

    //converting geodetic lonx, latx, alt to SBII
    SBII = arma_cad_in_geo84(lonx * RAD, latx * RAD, alt, get_rettime());

    //building inertial velocity
    TDI = arma_cad_tdi84(lonx * RAD, latx * RAD, alt, get_rettime());
    TGI = arma_cad_tgi84(lonx * RAD, latx * RAD, alt, get_rettime());
}

void Newton::load_geodetic_velocity(double alpha0x, double beta0x, double dvbe){
    //this->dvbe = dvbe;

    //building geodetic velocity VBED(3x1) from  alpha, beta, and dvbe
    arma::mat VBEB = this->build_VBEB(alpha0x, beta0x, dvbe);
    arma::mat33 TBD = kinematics->get_TBD_();
    //Geodetic velocity
    arma::mat VBED = trans(TBD) * VBEB;

    VBII = trans(TDI) * VBED + WEII * SBII;
}

void Newton::propagate(double int_step){
    double vmass = propulsion->get_vmass();
    arma::vec3 FAPB(forces->get_fapb_ptr());

    this->FSPB = calculate_fspb(FAPB, vmass);

    propagate_position_speed_acceleration(int_step);

    propagate_aeroloss(int_step);
    propagate_gravityloss(int_step);
}

arma::vec3 Newton::calculate_fspb(arma::vec3 FAPB, double vmass){
    /* Stored Value due to coherence with other models */
    return FAPB * (1. / vmass);
}

void Newton::propagate_position_speed_acceleration(double int_step){
    double lon, lat, al;

    arma::mat33 TBI(kinematics->get_TBI_());

    arma::vec3 GRAVG = environment->get_GRAVG_();

    /* Prograte S, V, A status */
    arma::mat NEXT_ACC = trans(TBI) * FSPB + trans(TGI) * GRAVG;
    arma::mat NEXT_VEL = integrate(NEXT_ACC, ABII, VBII, int_step);
    SBII = integrate(NEXT_VEL, VBII, SBII, int_step);
    ABII = NEXT_ACC;
    VBII = NEXT_VEL;

    //Calculate lon lat alt
    arma_cad_geo84_in(lon, lat, al, SBII, get_rettime());
    this->lonx = lon * DEG;
    this->latx = lat * DEG;
    this->alt  = al;

    TDI = arma_cad_tdi84(lon, lat, al, get_rettime());
    TGI = arma_cad_tgi84(lon, lat, al, get_rettime());
}

void Newton::propagate_aeroloss(double int_step){
    //XXX: Need fixing
    arma::vec3 FAP(forces->get_fap_ptr());

    //calculate aero loss`:`
    FAP = FAP * (1. / propulsion->get_vmass());
    aero_loss = aero_loss + norm(FAP) * int_step;
}

void Newton::propagate_gravityloss(double int_step){
    //calculate gravity loss
    gravity_loss = gravity_loss + environment->get_grav() * sin(get_thtvdx() * RAD) * int_step;
}

void Newton::update_diagnostic_attributes(double int_step){
    _dbi = get_dbi();
    _dvbi = get_dvbi();

    _dvbe = get_dvbe();
    _psivdx = get_psivdx();
    _thtvdx = get_thtvdx();

    //XXX: Need Fixing, if interface return type change
    //ground track travelled (10% accuracy, usually on the high side)
    double vbed1 = get_VBED()[0];
    double vbed2 = get_VBED()[1];
    _grndtrck += sqrt(vbed1 * vbed1 + vbed2 * vbed2) * int_step * REARTH / get_dbi();
    _gndtrkmx = 0.001 * _grndtrck;
    _gndtrnmx = NMILES * _grndtrck;

    _ayx =  FSPB(1) / AGRAV;
    _anx = -FSPB(2) / AGRAV;

    //T.M. of geographic velocity wrt geodetic coordinates
    arma::mat TVD(&_TVD[0][0], 3, 3, false, true);
    TVD = build_transform_matrix(_psivdx * RAD, _thtvdx * RAD);

    orbital(SBII, VBII, get_dbi());
}

void Newton::orbital(arma::vec3 SBII, arma::vec3 VBII, double dbi)
{
    //calculate orbital elements
    int cadorbin_flag = arma_cad_orb_in(_semi_major, _eccentricity, _inclination, _lon_anodex, _arg_perix, _true_anomx, SBII, VBII);
    _ha = (1. + _eccentricity) * _semi_major - REARTH;
    _hp = (1. - _eccentricity) * _semi_major - REARTH;
    _ref_alt = dbi - REARTH;
}

double Newton::get_alt() { return alt; }

double Newton::get_lonx() { return lonx; }

double Newton::get_latx() { return latx; }

Matrix Newton::get_IPos() { return Matrix(_SBII); }

Matrix Newton::get_IVel() { return Matrix(_VBII); }

/* Use stored Value due to coherence with other models */
Matrix Newton::get_FSPB() { return Matrix(_FSPB); }

Matrix Newton::get_VBED() {
    arma::mat VBED = TDI * (VBII - WEII * SBII);

    Matrix __VBED(VBED.memptr());
    return __VBED;
}

arma::vec Newton::get_VBED_() {
    arma::mat VBED = TDI * (VBII - WEII * SBII);

    return VBED;
}

double Newton::get_dbi() { return norm(SBII); }

double Newton::get_dvbi() { return norm(VBII); }

double Newton::get_dvbe(){
    Matrix VBED = get_VBED();
    return VBED.pol_from_cart().get_loc(0, 0);
}

double Newton::get_thtvdx(){
    Matrix VBED = get_VBED();
    return DEG * VBED.pol_from_cart().get_loc(2, 0);
}

double Newton::get_psivdx(){
    Matrix VBED = get_VBED();
    return DEG * VBED.pol_from_cart().get_loc(1, 0);
}

arma::vec Newton::get_VBII() { return VBII; }

arma::vec Newton::get_SBII() { return SBII; }
