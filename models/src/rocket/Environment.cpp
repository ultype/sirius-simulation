#include "aux/utility_header.hh"

#include "rocket/Environment.hh"
#include "sim_services/include/simtime.h"

#include "cad/env/atmosphere.hh"
#include "cad/env/atmosphere76.hh"
#include "cad/env/atmosphere_nasa2002.hh"
#include "cad/env/atmosphere_weatherdeck.hh"

#include "cad/env/wind.hh"
#include "cad/env/wind_no.hh"
#include "cad/env/wind_tabular.hh"
#include "cad/env/wind_constant.hh"

Environment::Environment(Newton &newt, AeroDynamics &aero, Kinematics &kine)
    :   newton(&newt), aerodynamics(&aero), kinematics(&kine),
        VECTOR_INIT(GRAVG, 3)
{
    this->default_data();

    atmosphere = NULL;
    wind       = NULL;
}

Environment::Environment(const Environment& other)
    :   newton(other.newton), aerodynamics(other.aerodynamics), kinematics(other.kinematics),
        VECTOR_INIT(GRAVG, 3)
{
    this->default_data();
}

Environment& Environment::operator=(const Environment& other) {
    if(&other == this)
        return *this;

    this->kinematics   = other.kinematics;
    this->newton       = other.newton;
    this->aerodynamics = other.aerodynamics;

    return *this;
}

Environment::~Environment() {
    if(atmosphere) delete atmosphere;
    if(wind) delete wind;
}

void Environment::initialize() {
    dvba = newton->get_dvbe();
}

void Environment::default_data() {
}

void Environment::atmosphere_use_public() {
    atmosphere = new cad::Atmosphere76();
}

void Environment::atmosphere_use_nasa() {
    atmosphere = new cad::Atmosphere_nasa2002();
}

void Environment::atmosphere_use_weather_deck(char* filename) {
    atmosphere = new cad::Atmosphere_weatherdeck(filename);
}

void Environment::set_no_wind(){
    wind = new cad::Wind_No();
}

void Environment::set_constant_wind(double dvae, double dir, double twind, double vertical_wind){
    wind = new cad::Wind_Constant(dvae, dir, twind, vertical_wind);
}

void Environment::set_tabular_wind(char* filename, double twind, double vertical_wind){
    wind = new cad::Wind_Tabular(filename, twind, vertical_wind);
}

void Environment::set_no_wind_turbulunce(){
    if(wind) wind->disable_turbulance();
}

void Environment::set_wind_turbulunce(double turb_length, double turb_sigma,
                                        double taux1, double taux1d,
                                        double taux2, double taux2d,
                                        double tau,   double gauss_value){
    if(wind) wind->enable_turbulance(turb_length, turb_sigma, taux1, taux1d, taux2, taux2d, tau, gauss_value);
}

void Environment::calculate_env(double int_step) {
    arma::vec3 VBED = newton->get_VBED_();
    // XXX: arma matrix subsitute
    Matrix SBII = newton->get_IPos();
    double alt = newton->get_alt();

    arma::mat TBD = kinematics->get_TBD_();
    double alppx = kinematics->get_alppx();
    double phipx = kinematics->get_phipx();

    this->GRAVG = arma::vec3(cad_grav84(SBII, get_rettime()).get_pbody());
    this->grav = norm(GRAVG);

    atmosphere->set_altitude(alt);

    rho    = atmosphere->get_density();
    press  = atmosphere->get_pressure();
    tempk  = atmosphere->get_temperature_in_kelvin();
    vsound = atmosphere->get_speed_of_sound();

    tempc = tempk - 273.16;

    wind->set_altitude(alt);
    psiwdx = wind->get_direction_of_wind();
    wind->propagate_VAED(int_step);
    wind->apply_turbulance_if_have(int_step, dvba, TBD, alppx, phipx);

    //flight conditions
    arma::vec3 VBAD = VBED - wind->get_VAED();
    dvba = norm(VBAD);

    //mach number
    vmach = fabs(dvba / vsound);

    //dynamic pressure
    pdynmc = 0.5 * rho * dvba * dvba;
}

double Environment::get_rho() { return rho; }
double Environment::get_vmach() { return vmach; }
double Environment::get_pdynmc() { return pdynmc; }
double Environment::get_tempk() { return tempk; }
double Environment::get_dvba() { return dvba; }
double Environment::get_grav() { return grav; }
double Environment::get_press() { return press; }
arma::vec3 Environment::get_GRAVG_() { return GRAVG; }
arma::vec3 Environment::get_VAED_() { return wind->get_VAED(); }

Matrix Environment::get_GRAVG() {
    Matrix GRAVG(_GRAVG);
    return GRAVG;
}
Matrix Environment::get_VAED() {
    Matrix VAED(wind->get_VAED().memptr());
    return VAED;
}

