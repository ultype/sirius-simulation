#include "aux/utility_header.hh"

#include "rocket/Propulsion.hh"
#include "sim_services/include/simtime.h"


void Propulsion::initialize(Environment *env)
{
    environment=env;
}

void Propulsion::calculate_propulsion(double int_step)
{
    double psl(101300); //chamber pressure - Pa
    Matrix IBBB0(3,3);
    Matrix IBBB1(3,3);
    Matrix IBBB(3,3);

    double press=environment->get_press();

    //no thrusting
    if(mprop==0){
        fmassd=0;
        thrust=0;
        fmasse=0;
        //fmassr=0;
        vmass=payload+vmass0-fmasse;

    }

    //making thrust calculations only if engine is on
    if(mprop>0){

        //constant thrust rocket engine for booster
        if(mprop==3||mprop==4){
            thrust=spi*fuel_flow_rate*AGRAV+(psl-press)*aexit;
            //load MOI of booster
            IBBB0.zero();
            IBBB0.assign_loc(0,0,moi_roll_0);
            IBBB0.assign_loc(1,1,moi_trans_0);
            IBBB0.assign_loc(2,2,moi_trans_0);
            IBBB1.zero();
            IBBB1.assign_loc(0,0,moi_roll_1);
            IBBB1.assign_loc(1,1,moi_trans_1);
            IBBB1.assign_loc(2,2,moi_trans_1);
        }
        //calculating fuel consumption
        if (spi!=0){
            double fmassd_next=(thrust-(psl-press)*aexit)/(spi*AGRAV);//thrust/(spi*AGRAV)
            fmasse=integrate(fmassd_next,fmassd,fmasse,int_step);
            fmassd=fmassd_next;
        }
        //calculating vehicle mass, mass flow, and fuel mass remaining
        vmass=payload+vmass0-fmasse;

        fmassr=fmass0-fmasse;

        //calculating delta-v

        thrust_delta_v=thrust_delta_v+spi*AGRAV*(fuel_flow_rate/vmass)*int_step;


        //interpolating moment of inertia tensor as a function of fuel expended
        double mass_ratio=fmasse/fmass0;
        IBBB=IBBB0+(IBBB1-IBBB0)*mass_ratio;

        //interpolating cg as a function of fuel expended
        xcg=xcg_0+(xcg_1-xcg_0)*mass_ratio;
        //shutting down engine when all fuel is expended
        if(fmassr<=0){
            mprop=0;
            thrust=0;
        }
        IBBB.fill(ibbb);
    }
}

void Propulsion::set_vmass0(double in) { vmass0 = in; }
void Propulsion::set_fmass0(double in) { fmass0 = in; }
void Propulsion::set_xcg_0(double in) { xcg_0 = in; }
void Propulsion::set_xcg_1(double in) { xcg_1 = in; }
void Propulsion::set_fuel_flow_rate(double in) { fuel_flow_rate = in; }
void Propulsion::set_moi_roll_0(double in) { moi_roll_0 = in; }
void Propulsion::set_moi_roll_1(double in) { moi_roll_1 = in; }
void Propulsion::set_moi_trans_0(double in) { moi_trans_0 = in; }
void Propulsion::set_moi_trans_1(double in) { moi_trans_1 = in; }
void Propulsion::set_aexit(double in) { aexit = in; }
void Propulsion::set_spi(double in) { spi = in; }
void Propulsion::set_payload(double in) { payload = in; }
void Propulsion::set_fmasse(double in) { fmasse = in; }

int Propulsion::get_mprop() { return mprop; }
void Propulsion::set_mprop(int in) { mprop = in; }
double Propulsion::get_vmass() { return vmass; }
double Propulsion::get_xcg() { return xcg; }
double Propulsion::get_thrust() { return thrust; }
double Propulsion::get_fmassr() { return fmassr; }
Matrix Propulsion::get_IBBB() {
    Matrix IBBB(3, 3);
    IBBB.build_mat33(ibbb);
    return IBBB;
}
