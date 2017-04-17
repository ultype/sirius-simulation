#include "rocket/Newton.hh"
#include "sim_services/include/simtime.h"

#include "math/integrate.hh"
#include "math/matrix/utility.hh"

#include "cad/utility.hh"

#include "aux/aux.hh"

Newton::Newton(Kinematics &kine, _Euler_ &elr, Environment &env, Propulsion &prop, Forces &forc)
    :   kinematics(&kine), euler(&elr), environment(&env), propulsion(&prop), forces(&forc),
        MATRIX_INIT(WEII, 3, 3),
        MATRIX_INIT(TDI, 3, 3),
        MATRIX_INIT(TGI, 3, 3),
        VECTOR_INIT(SBII, 3),
        VECTOR_INIT(VBII, 3),
        VECTOR_INIT(ABII, 3),
        VECTOR_INIT(FSPB, 3),
        VECTOR_INIT(ABIB, 3),
        VECTOR_INIT(SBEE, 3),
        VECTOR_INIT(VBEE, 3)
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
        VECTOR_INIT(FSPB, 3),
        VECTOR_INIT(ABIB, 3),
        VECTOR_INIT(SBEE, 3),
        VECTOR_INIT(VBEE, 3)
{
    this->default_data();

    /* Propagative Stats */
    this->alt = other.alt;
    this->lonx = other.lonx;
    this->latx = other.latx;
    this->SBII = other.SBII;
    this->VBII = other.VBII;
    this->ABII = other.ABII;
    this->ABIB = other.ABIB;

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
    this->ABIB = other.ABIB;

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
    this->ABII = this->WEII * (this->WEII * this->SBII);
    arma::mat33 TEI = environment->get_TEI();//cad::tei(get_rettime());
    SBEE = TEI * SBII; //Calculate position in ECEF
    liftoff = 0;
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
    SBII = cad::in_geo84(lonx * RAD, latx * RAD, alt, get_rettime());

    //building inertial velocity
    TDI = cad::tdi84(lonx * RAD, latx * RAD, alt, get_rettime());
    TGI = cad::tgi84(lonx * RAD, latx * RAD, alt, get_rettime());
}

void Newton::load_geodetic_velocity(double alpha0x, double beta0x, double dvbe){
    //this->dvbe = dvbe;

    //building geodetic velocity VBED(3x1) from  alpha, beta, and dvbe
    arma::mat VBEB = this->build_VBEB(alpha0x, beta0x, dvbe);
    arma::mat33 TBD = kinematics->get_TBD();
    //Geodetic velocity
    arma::mat VBED = trans(TBD) * VBEB;

    VBII = trans(TDI) * VBED + WEII * SBII;
}

void Newton::propagate(double int_step){
    double vmass = propulsion->get_vmass();
    arma::vec3 FAPB = forces->get_FAPB();

    this->FSPB = calculate_fspb(FAPB, vmass);

    propagate_position_speed_acceleration(int_step);
    if(liftoff == 1){
        propagate_aeroloss(int_step);
        propagate_gravityloss(int_step);
    }

}

arma::vec3 Newton::calculate_fspb(arma::vec3 FAPB, double vmass){
    /* Stored Value due to coherence with other models */
    return FAPB * (1. / vmass);
}

void Newton::propagate_position_speed_acceleration(double int_step){
    double lon, lat, al;

    arma::mat33 TBI = kinematics->get_TBI();

    arma::vec3 GRAVG = environment->get_GRAVG();

    arma::mat33 TEI = environment->get_TEI();//cad::tei(get_rettime());

    arma::vec3 WEII = euler->get_WEII();

    /* Prograte S, V, A status */
    arma::mat NEXT_ACC = trans(TBI) * FSPB + trans(TGI) * GRAVG;
    /* To check wether the rocket liftoff */
    if(liftoff==0){
        if(norm(FSPB) - norm(GRAVG)>0){
            liftoff = 1;
        }else{
            NEXT_ACC = this->WEII * (this->WEII * this->SBII);
        }

    }

    arma::mat NEXT_VEL = integrate(NEXT_ACC, ABII, VBII, int_step);
    SBII = integrate(NEXT_VEL, VBII, SBII, int_step);
    ABII = NEXT_ACC;
    VBII = NEXT_VEL;
    ABIB = TBI * ABII;

    SBEE = TEI * SBII; //Calculate position in ECEF
    VBEE = TEI * VBII - cross(WEII, SBEE); //Calculate velocity in ECEF 

    //Calculate lon lat alt
    cad::geo84_in(lon, lat, al, SBII, get_rettime());
    this->lonx = lon * DEG;
    this->latx = lat * DEG;
    this->alt  = al;

    TDI = cad::tdi84(lon, lat, al, get_rettime());
    TGI = cad::tgi84(lon, lat, al, get_rettime());
}

void Newton::propagate_aeroloss(double int_step){
    arma::vec3 FAP = forces->get_FAP();

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

    if(liftoff == 1){
        //T.M. of geographic velocity wrt geodetic coordinates
        arma::mat TVD(&_TVD[0][0], 3, 3, false, true);
        TVD = build_psivg_thtvg_TM(_psivdx * RAD, _thtvdx * RAD);

        orbital(SBII, VBII, get_dbi());
    }
}

void Newton::orbital(arma::vec3 SBII, arma::vec3 VBII, double dbi)
{
    //calculate orbital elements
    int cadorbin_flag = cad::orb_in(_semi_major, _eccentricity, _inclination, _lon_anodex, _arg_perix, _true_anomx, SBII, VBII);
    _ha = (1. + _eccentricity) * _semi_major - REARTH;
    _hp = (1. - _eccentricity) * _semi_major - REARTH;
    _ref_alt = dbi - REARTH;
}

double Newton::get_alt() { return alt; }
double Newton::get_lonx() { return lonx; }
double Newton::get_latx() { return latx; }

double Newton::get_dbi() { return norm(SBII); }
double Newton::get_dvbi() { return norm(VBII); }
double Newton::get_dvbe() { return pol_from_cart(get_VBED())(0); }
double Newton::get_thtvdx(){ return DEG * pol_from_cart(get_VBED())(2); }
double Newton::get_psivdx(){ return DEG * pol_from_cart(get_VBED())(1); }

arma::vec3 Newton::get_VBED() { return TDI * (VBII - WEII * SBII); }
arma::vec3 Newton::get_VBII() { return VBII; }
arma::vec3 Newton::get_SBII() { return SBII; }
/* Use stored Value due to coherence with other models */
arma::vec3 Newton::get_FSPB() { return FSPB; }
arma::vec3 Newton::get_SBEE() { return SBEE; }

unsigned int Newton::get_liftoff() { return liftoff; }

arma::mat Newton::get_TGI() { return TGI; }
arma::vec3 Newton::get_VBEE() { return VBEE; }
