#include "Newton.hh"
#include "trick_utils/math/include/quat_macros.h"
#include "trick_utils/math/include/matrix_macros.h"
#include "trick_utils/math/include/vector_macros.h"
#include "trick_utils/math/include/trick_math_proto.h"
#include "sim_services/include/simtime.h"

#include "utility_header.hh"


double Newton::get_alt(){
    return alt;
}

double Newton::get_lonx(){
    return lonx;
}

double Newton::get_latx(){
    return latx;
}

Matrix Newton::get_IPos(){
    return Matrix(_SBII);
}

Matrix Newton::get_IVel(){
    return Matrix(_VBII);
}

Matrix Newton::get_FSPB(){
    /* Use stored Value due to coherence with other models */
    return Matrix(_FSPB);
}

Matrix Newton::get_VBED(){
    arma::mat VBED = TDI * (VBII - WEII * SBII);

    Matrix __VBED(VBED.memptr());
    return __VBED;
}

double Newton::get_dbi(){
    return norm(SBII);
}

double Newton::get_dvbi(){
    return norm(VBII);
}

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

Newton::Newton(Kinematics &kine, _Euler_ &elr, Environment &env, Propulsion &prop, Forces &forc)
    :   kinematics(&kine), euler(&elr), environment(&env), propulsion(&prop), forces(&forc),
        WEII(&_WEII[0][0], 3, 3, false, true),
        SBII(&_SBII[0], 3, false, true),
        VBII(&_VBII[0], 3, false, true),
        ABII(&_ABII[0], 3, false, true),
        FSPB(&_FSPB[0], 3, false, true),
        TDI(&_TDI[0][0], 3, 3, false, true),
        TGI(&_TGI[0][0], 3, 3, false, true)
{
    this->default_data();
}

Newton::Newton(const Newton& other)
    :   kinematics(other.kinematics), euler(other.euler), environment(other.environment), propulsion(other.propulsion), forces(other.forces),
        WEII(&_WEII[0][0], 3, 3, false, true),
        SBII(&_SBII[0], 3, false, true),
        VBII(&_VBII[0], 3, false, true),
        ABII(&_ABII[0], 3, false, true),
        FSPB(&_FSPB[0], 3, false, true),
        TDI(&_TDI[0][0], 3, 3, false, true),
        TGI(&_TGI[0][0], 3, 3, false, true)
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

    //XXX: Need fixing
    //converting geodetic lonx, latx, alt to SBII
    Matrix SBII = cad_in_geo84(lonx * RAD, latx * RAD, alt, get_rettime());
    SBII.fill(this->_SBII);

    //building inertial velocity
    Matrix TDI = cad_tdi84(lonx * RAD, latx * RAD, alt, get_rettime());
    Matrix TGI = cad_tgi84(lonx * RAD, latx * RAD, alt, get_rettime());
    TDI.trans().fill(this->_TDI);
    TGI.trans().fill(this->_TGI);
}

void Newton::load_geodetic_velocity(double alpha0x, double beta0x, double dvbe){
    //this->dvbe = dvbe;

    //XXX: Need fixing
    //building geodetic velocity VBED(3x1) from  alpha, beta, and dvbe
    arma::mat VBEB = this->build_VBEB(alpha0x, beta0x, dvbe);
    //building TBD
    //XXX: Need Fix
    arma::mat33 TBD(mat3tr(kinematics->psibdx * RAD, kinematics->thtbdx * RAD, kinematics->phibdx * RAD).get_pbody());
    TBD = trans(TBD);
    //Geodetic velocity
    arma::mat VBED = trans(TBD) * VBEB;

    VBII = trans(TDI) * VBED + WEII * SBII;
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

void Newton::propagate(double int_step){
    update_fspb();

    propagate_position_speed_acceleration(int_step);

    propagate_aeroloss(int_step);
    propagate_gravityloss(int_step);
}

void Newton::update_fspb(){
    arma::vec3 FAPB(&forces->fapb[0]);

    /* Stored Value due to coherence with other models */
    FSPB = FAPB * (1. / propulsion->vmass);
}

void Newton::propagate_position_speed_acceleration(double int_step){
    double lon, lat, al;

    //XXX: Need Fix
    arma::mat33 TBI(&kinematics->tbi[0][0]);
    TBI = trans(TBI);

    arma::vec3 GRAVG(&environment->gravg[0]);

    /* Prograte S, V, A status */
    arma::mat NEXT_ACC = trans(TBI) * FSPB + trans(TGI) * GRAVG;
    arma::mat NEXT_VEL = integrate(NEXT_ACC, ABII, VBII, int_step);
    SBII = integrate(NEXT_VEL, VBII, SBII, int_step);
    ABII = NEXT_ACC;
    VBII = NEXT_VEL;


    //XXX: Temp fix
    // Load old Matrix with aux memory
    Matrix __SBII(_SBII);

    //Calculate lon lat alt
    cad_geo84_in(lon, lat, al, __SBII, get_rettime());

    this->lonx = lon * DEG;
    this->latx = lat * DEG;
    this->alt  = al;

    // Use old Metrix Type
    Matrix __TDI = cad_tdi84(lon, lat, al, get_rettime());
    Matrix __TGI = cad_tgi84(lon, lat, al, get_rettime());

    //XXX: Need Fix
    //Fill back into aux memory
    __TDI = ~__TDI;
    __TGI = ~__TGI;
    __TDI.fill(_TDI);
    __TGI.fill(_TGI);
}

void Newton::propagate_aeroloss(double int_step){
    //XXX: Need fixing

    arma::vec3 FAP(&forces->fap[0]);

    //calculate aero loss`:`
    FAP = FAP * (1. / propulsion->vmass);
    aero_loss = aero_loss + norm(FAP) * int_step;
}

void Newton::propagate_gravityloss(double int_step){
    //calculate gravity loss
    gravity_loss = gravity_loss + environment->grav * sin(get_thtvdx() * RAD) * int_step;
}

void Newton::update_diagnostic_attributes(double int_step){
    _dbi = get_dbi();
    _dvbi = get_dvbi();

    //XXX: Need Fixing
    // Load up old Old Matrix Type from aux memory
    Matrix VBED = get_VBED();

    Matrix POLAR = VBED.pol_from_cart();
    _dvbe = POLAR[0];
    _psivdx = DEG * POLAR[1];
    _thtvdx = DEG * POLAR[2];

    //ground track travelled (10% accuracy, usually on the high side)
    double vbed1 = VBED[0];
    double vbed2 = VBED[1];
    _grndtrck += sqrt(vbed1 * vbed1 + vbed2 * vbed2) * int_step * REARTH / get_dbi();
    _gndtrkmx = 0.001 * _grndtrck;
    _gndtrnmx = NMILES * _grndtrck;

    //XXX: Need Fixing
    // Load up old Old Matrix Type from aux memory
    Matrix FSPB(_FSPB);
    _ayx =  FSPB[1] / AGRAV;
    _anx = -FSPB[2] / AGRAV;

    //XXX: Need Fixing
    //T.M. of geographic velocity wrt geodetic coordinates
    Matrix TVD = mat2tr(_psivdx * RAD, _thtvdx * RAD);
    TVD = ~TVD;
    TVD.fill(_TVD);

    //XXX: Need Fixing
    Matrix SBII(_SBII);
    Matrix VBII(_VBII);

    orbital(SBII,VBII,get_dbi());
}

void Newton::orbital(Matrix &SBII, Matrix &VBII, double dbi)
{
    //calculate orbital elements
    int cadorbin_flag = cad_orb_in(_semi_major, _eccentricity, _inclination, _lon_anodex, _arg_perix, _true_anomx, SBII, VBII);
    _ha = (1. + _eccentricity) * _semi_major - REARTH;
    _hp = (1. - _eccentricity) * _semi_major - REARTH;
    _ref_alt = dbi - REARTH;

}

