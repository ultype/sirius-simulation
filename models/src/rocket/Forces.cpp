#include "rocket/Force.hh"
#include "sim_services/include/simtime.h"

#include "aux/aux.hh"

Forces::Forces(Environment& env, Propulsion& prop, RCS& rcs, AeroDynamics& aero, TVC& tvc)
    :   environment(&env), propulsion(&prop), rcs(&rcs), aerodynamics(&aero), tvc(&tvc),
        VECTOR_INIT(FAPB, 3),
        VECTOR_INIT(FAP, 3),
        VECTOR_INIT(FMB, 3)
{
    this->default_data();
}
Forces::Forces(const Forces& other)
    :   environment(other.environment), propulsion(other.propulsion), rcs(other.rcs), aerodynamics(other.aerodynamics), tvc(other.tvc),
        VECTOR_INIT(FAPB, 3),
        VECTOR_INIT(FAP, 3),
        VECTOR_INIT(FMB, 3)
{
    this->default_data();
}

Forces& Forces::operator=(const Forces& other){
    if(&other == this)
        return *this;

    this->environment = other.environment;
    this->propulsion = other.propulsion;
    this->rcs = other.rcs;
    this->aerodynamics = other.aerodynamics;
    this->tvc = other.tvc;

    return *this;
}

void Forces::default_data(){
}


void Forces::initialize()
{
}

void Forces::collect_forces_and_propagate(){
    /*****************input from another module*******************/
    double pdynmc = environment->get_pdynmc();

    enum Propulsion::THRUST_TYPE thrust_type = propulsion->get_thrust_state();
    double thrust                            = propulsion->get_thrust();

    bool rcs_enabled            = rcs->isEnabled();
    enum RCS::RCS_MODE rcs_mode = rcs->get_rcs_mode();
    arma::vec3 FMRCS            = rcs->get_FMRCS_();
    arma::vec3 FARCS            = rcs->get_FARCS_();

    double refa = aerodynamics->get_refa();
    double refd = aerodynamics->get_refd();
    double cy   = aerodynamics->get_cy();
    double cll  = aerodynamics->get_cll();
    double clm  = aerodynamics->get_clm();
    double cln  = aerodynamics->get_cln();
    double cx   = aerodynamics->get_cx();
    double cz   = aerodynamics->get_cz();

    enum TVC::TVC_TYPE mtvc = tvc->get_mtvc();
    arma::vec3 FPB          = tvc->get_FPB_();
    arma::vec3 FMPB         = tvc->get_FMPB_();
    /*************************************************************/

    //total non-gravitational forces
    FAPB(0, 0) = pdynmc*refa*cx;
    FAPB(1, 0) = pdynmc*refa*cy;
    FAPB(2, 0) = pdynmc*refa*cz;

    //aerodynamic loss
    FAP(0, 0) = pdynmc*refa*cx;
    FAP(1, 0) = pdynmc*refa*cy;
    FAP(2, 0) = pdynmc*refa*cz;

    //aerodynamic moment
    FMB(0, 0) = pdynmc*refa*refd*cll;
    FMB(1, 0) = pdynmc*refa*refd*clm;
    FMB(2, 0) = pdynmc*refa*refd*cln;

    //adding thrust modified by TVC or otherwise just plain thrust if mprop > 0
    if(mtvc != TVC::NO_TVC){
        FAPB=FAPB+FPB;
        FMB=FMB+FMPB;
    }
    else if(thrust_type != Propulsion::NO_THRUST)
        FAPB[0]=FAPB[0]+thrust;

    //adding force components from RCS
    //if(rcs_enabled && (rcs_mode == ALL_GEODETIC_EULUR_ANGLE_CONTROL || rcs_mode = THRUST_VECTOR_DIRECTION_AND_ROLL_ANGLE_CONTROL))
        //FAPB=FAPB+FARCS;

    //adding moment components from RCS
    if(rcs_enabled && rcs_mode != RCS::GEODETIC_YAW_ANGLE_CONTROL)
        FMB=FMB+FMRCS;

}

double* Forces::get_fapb_ptr() { return _FAPB; }
double* Forces::get_fap_ptr() { return _FAP; }

Matrix Forces::get_FAPB() {
    Matrix FAPB(_FAPB);
    return FAPB;
}

Matrix Forces::get_FAP() {
    Matrix FAP(_FAP);
    return FAP;
}

Matrix Forces::get_FMB() {
    Matrix FMB(_FMB);
    return FMB;
}

arma::vec Forces::get_FAPB_() { return FAPB; }
arma::vec Forces::get_FAP_() { return FAP; }
arma::vec Forces::get_FMB_() { return FMB; }
