#include "aux/utility_header.hh"

#include "rocket/Propulsion.hh"
#include "sim_services/include/simtime.h"

Propulsion::Propulsion(Environment& env)
    :   environment(&env)
{
    this->default_data();
}

Propulsion::Propulsion(const Propulsion& other)
    :   environment(other.environment)
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

void Propulsion::set_ltg_thrust(){
    this->thrust_state = LTG_THRUST;
}

void Propulsion::calculate_propulsion(double int_step)
{
    double psl(101300); //chamber pressure - Pa
    Matrix IBBB0(3,3);
    Matrix IBBB1(3,3);
    Matrix IBBB(3,3);

    double press=environment->get_press();

    //no thrusting
    switch(this->thrust_state){
        case NO_THRUST:
            fmassd = 0;
            thrust = 0;
            fmasse = 0;
            //fmassr=0; //Somehow comment out in trick version

            break;
        case INPUT_THRUST:
        case LTG_THRUST:

            this->thrust = this->spi * this->fuel_flow_rate * AGRAV + (psl - press) * this->aexit;

            //calculating delta-v
            this->thrust_delta_v += this->spi * AGRAV * (this->fuel_flow_rate / this->vmass) * int_step;

            //calculating fuel consumption
            if (spi != 0){
                double fmassd_next = (this->thrust - (psl - press) * this->aexit) / (this->spi * AGRAV); //thrust/(spi*AGRAV)
                this->fmasse = integrate(fmassd_next, this->fmassd, this->fmasse, int_step);
                this->fmassd = fmassd_next;
            }

            this->fmassr = this->fmass0 - this->fmasse;

            //shutting down engine when all fuel is expended
            if(fmassr <= 0){
                this->thrust_state = NO_THRUST;
                this->thrust = 0;
            }

            //load MOI of booster
            IBBB0.zero();
            IBBB0.assign_loc(0,0,moi_roll_0);
            IBBB0.assign_loc(1,1,moi_trans_0);
            IBBB0.assign_loc(2,2,moi_trans_0);
            IBBB1.zero();
            IBBB1.assign_loc(0,0,moi_roll_1);
            IBBB1.assign_loc(1,1,moi_trans_1);
            IBBB1.assign_loc(2,2,moi_trans_1);
            //interpolating moment of inertia tensor as a function of fuel expended
            double mass_ratio = fmasse / fmass0;
            IBBB = IBBB0 + (IBBB1 - IBBB0) * mass_ratio;

            //interpolating cg as a function of fuel expended
            this->xcg = xcg_0 + (xcg_1 - xcg_0) * mass_ratio;
            IBBB.fill(ibbb);

            break;
    }

    //calculating vehicle mass, mass flow, and fuel mass remaining
    vmass = payload + vmass0 - fmasse;
}

void Propulsion::set_vmass0(double in) { vmass0 = in; }
void Propulsion::set_fmass0(double in) { fmass0 = in; }
void Propulsion::set_aexit(double in) { aexit = in; }
void Propulsion::set_payload(double in) { payload = in; }

int Propulsion::get_mprop() { return (int)this->thrust_state; /*XXX work around*/ }
double Propulsion::get_vmass() { return vmass; }
double Propulsion::get_xcg() { return xcg; }
double Propulsion::get_thrust() { return thrust; }
double Propulsion::get_fmassr() { return fmassr; }
Matrix Propulsion::get_IBBB() {
    Matrix IBBB(3, 3);
    IBBB.build_mat33(ibbb);
    return IBBB;
}
