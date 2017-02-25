#include "aux/utility_header.hh"

#include "rocket/Propulsion.hh"
#include "sim_services/include/simtime.h"

Propulsion::Propulsion(Environment& env)
    :   environment(&env),
        MATRIX_INIT(IBBB, 3, 3)
{
    this->default_data();
}

Propulsion::Propulsion(const Propulsion& other)
    :   environment(other.environment),
        MATRIX_INIT(IBBB, 3, 3)
{
    this->default_data();

    /* Propagative Stats */
}

Propulsion& Propulsion::operator=(const Propulsion& other){
    if(&other == this)
        return *this;

    this->environment = other.environment;

    return *this;
}

void Propulsion::initialize(){
}

void Propulsion::default_data(){
    this->fmassd = 0;
    this->thrust = 0;
    this->fmasse = 0;

    this->payload = 0;
}

void Propulsion::set_no_thrust(){
    this->thrust_state = NO_THRUST;
}

void Propulsion::set_input_thrust(double xcg0, double xcg1,
                                    double moi_roll0, double moi_roll1,
                                    double moi_trans0, double moi_trans1,
                                    double spi, double fuel_flow_rate)
{
    this->thrust_state = INPUT_THRUST;

    this->fmasse = 0;

    this->xcg_0          = xcg0;
    this->xcg_1          = xcg1;
    this->moi_roll_0     = moi_roll0;
    this->moi_roll_1     = moi_roll1;
    this->moi_trans_0    = moi_trans0;
    this->moi_trans_1    = moi_trans1;
    this->fuel_flow_rate = fuel_flow_rate;
    this->spi            = spi;
}

void Propulsion::get_input_file_var(double xcg0, double xcg1,
                                    double moi_roll0, double moi_roll1,
                                    double moi_trans0, double moi_trans1,
                                    double spi, double fuel_flow_rate)
{
    this->xcg_0          = xcg0;
    this->xcg_1          = xcg1;
    this->moi_roll_0     = moi_roll0;
    this->moi_roll_1     = moi_roll1;
    this->moi_trans_0    = moi_trans0;
    this->moi_trans_1    = moi_trans1;
    this->fuel_flow_rate = fuel_flow_rate;
    this->spi            = spi;
}

void Propulsion::set_ltg_thrust(){
    this->thrust_state = LTG_THRUST;
}

void Propulsion::propagate(double int_step)
{
    double psl(101300); //chamber pressure - Pa
    arma::mat33 IBBB0;
    arma::mat33 IBBB1;

    double press = environment->get_press();

    //no thrusting
    switch(this->thrust_state){
        case NO_THRUST:
            fmassd = 0;
            thrust = 0;
            fmasse = 0;
            //fmassr=0; //Somehow comment out in trick version
            this->IBBB = get_IBBB0();

            break;
        case INPUT_THRUST:
        case LTG_THRUST:

            this->thrust = calculate_thrust(press, psl);

            //calculating delta-v
            propagate_thrust_delta_v(int_step);

            //calculating fuel consumption
            if (spi != 0){
                propagate_fmasse(int_step, press, psl);
            }

            this->fmassr = calculate_fmassr();

            //shutting down engine when all fuel is expended
            if(fmassr <= 0){
                this->thrust_state = NO_THRUST;
                this->thrust = 0;
            }

            //interpolating cg as a function of fuel expended
            this->xcg = calculate_xcg();

            //interpolating moment of inertia tensor as a function of fuel expended
            this->IBBB = calculate_IBBB();

            break;
    }

    //calculating vehicle mass, mass flow, and fuel mass remaining
    vmass = payload + vmass0 - fmasse;
}

void Propulsion::propagate_thrust_delta_v(double int_step){
    this->thrust_delta_v += this->spi * AGRAV * (this->fuel_flow_rate / this->vmass) * int_step;
}

void Propulsion::propagate_fmasse(double int_step, double press, double psl){
    double fmassd_next = (this->thrust - (psl - press) * this->aexit) / (this->spi * AGRAV); //thrust/(spi*AGRAV)
    this->fmasse = integrate(fmassd_next, this->fmassd, this->fmasse, int_step);
    this->fmassd = fmassd_next;
}
double Propulsion::calculate_thrust(double press, double psl){
    return this->spi * this->fuel_flow_rate * AGRAV + (psl - press) * this->aexit;
}

double Propulsion::calculate_fmassr(){
    return this->fmass0 - this->fmasse;
}

double Propulsion::calculate_xcg(){
    double mass_ratio = fmasse / fmass0;
    return xcg_0 + (xcg_1 - xcg_0) * mass_ratio;
}

arma::mat33 Propulsion::calculate_IBBB(){
    double mass_ratio = fmasse / fmass0;
    return get_IBBB0() + (get_IBBB1() - get_IBBB0()) * mass_ratio;
}

arma::mat33 Propulsion::get_IBBB0(){
    arma::mat33 IBBB0(arma::fill::zeros);
    IBBB0(0,0) = moi_roll_0;
    IBBB0(1,1) = moi_trans_0;
    IBBB0(2,2) = moi_trans_0;

    return IBBB0;
}

arma::mat33 Propulsion::get_IBBB1(){
    arma::mat33 IBBB1(arma::fill::zeros);
    IBBB1(0,0) = moi_roll_1;
    IBBB1(1,1) = moi_trans_1;
    IBBB1(2,2) = moi_trans_1;

    return IBBB1;
}

void Propulsion::set_vmass0(double in) { vmass0 = in; }
void Propulsion::set_fmass0(double in) { fmass0 = in; }
void Propulsion::set_aexit(double in) { aexit = in; }
void Propulsion::set_payload(double in) { payload = in; }

int Propulsion::get_mprop() { return (int)this->thrust_state; /*XXX work around*/ }
enum Propulsion::THRUST_TYPE Propulsion::get_thrust_state() { return this->thrust_state; }
double Propulsion::get_vmass() { return vmass; }
double Propulsion::get_xcg() { return xcg; }
double Propulsion::get_thrust() { return thrust; }
double Propulsion::get_fmassr() { return fmassr; }

Matrix Propulsion::get_IBBB() {
    Matrix IBBB(_IBBB);
    return IBBB;
}

arma::mat33 Propulsion::get_IBBB_() {
    return IBBB;
}
