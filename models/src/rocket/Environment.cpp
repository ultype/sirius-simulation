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
        VECTOR_INIT(GRAVG, 3),
        atmosphere(new cad::Atmosphere(*other.atmosphere)),
        wind(new cad::Wind(*other.wind))
{
    this->default_data();

    this->GRAVG = other.GRAVG;
    this->vmach = other.vmach;
    this->pdynmc = other.pdynmc;
    this->dvba = other.dvba;

}

Environment& Environment::operator=(const Environment& other) {
    if(&other == this)
        return *this;

    this->kinematics   = other.kinematics;
    this->newton       = other.newton;
    this->aerodynamics = other.aerodynamics;

    this->atmosphere = new cad::Atmosphere(*other.atmosphere);
    this->wind = new cad::Wind(*other.wind);

    this->GRAVG = other.GRAVG;
    this->vmach = other.vmach;
    this->pdynmc = other.pdynmc;
    this->dvba = other.dvba;

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

void Environment::propagate(double int_step) {
    arma::vec3 VBED = newton->get_VBED_();
    arma::vec3 SBII = newton->get_SBII();
    double alt = newton->get_alt();

    arma::mat TBD = kinematics->get_TBD_();
    double alppx = kinematics->get_alppx();
    double phipx = kinematics->get_phipx();

    this->GRAVG = arma_cad_grav84(SBII, get_rettime());

    atmosphere->set_altitude(alt);

    wind->set_altitude(alt);
    wind->propagate_VAED(int_step);
    wind->apply_turbulance_if_have(int_step, dvba, TBD, alppx, phipx);

    //flight conditionsnorm(VBAD)
    arma::vec3 VBAD = VBED - wind->get_VAED();
    this->dvba = norm(VBAD);

    //mach number
    this->vmach = fabs(this->dvba / atmosphere->get_speed_of_sound());

    //dynamic pressure
    this->pdynmc = 0.5 * atmosphere->get_density() * this->dvba * this->dvba;
}

void Environment::update_diagnostic_attributes(double int_step) {
    this->gravg = get_grav();
    this->tempc = atmosphere->get_temperature_in_kelvin() + 273.16;
}

double Environment::get_press() { return atmosphere->get_pressure(); }
double Environment::get_rho() { return atmosphere->get_density(); }
double Environment::get_tempk() { return atmosphere->get_temperature_in_kelvin(); }

double Environment::get_vmach() { return vmach; }
double Environment::get_pdynmc() { return pdynmc; }
double Environment::get_dvba() { return dvba; }

double Environment::get_grav() { return norm(GRAVG); }

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

