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

    this->FAP = other.FAP;
    this->FAPB = other.FAPB;
    this->FMB = other.FMB;
}

Forces& Forces::operator=(const Forces& other){
    if(&other == this)
        return *this;

    this->environment = other.environment;
    this->propulsion = other.propulsion;
    this->rcs = other.rcs;
    this->aerodynamics = other.aerodynamics;
    this->tvc = other.tvc;

    this->FAP = other.FAP;
    this->FAPB = other.FAPB;
    this->FMB = other.FMB;

    return *this;
}

void Forces::default_data(){
}


void Forces::initialize(){
}

void Forces::collect_forces_and_propagate(){
    /*****************input from another module*******************/
    double pdynmc = environment->get_pdynmc();

    enum Propulsion::THRUST_TYPE thrust_type = propulsion->get_thrust_state();
    double thrust                            = propulsion->get_thrust();

    bool rcs_enabled            = rcs->isEnabled();
    enum RCS::RCS_MODE rcs_mode = rcs->get_rcs_mode();
    arma::vec3 FMRCS            = rcs->get_FMRCS();
    arma::vec3 FARCS            = rcs->get_FARCS();

    double refa = aerodynamics->get_refa();
    double refd = aerodynamics->get_refd();
    double cy   = aerodynamics->get_cy();
    double cll  = aerodynamics->get_cll();
    double clm  = aerodynamics->get_clm();
    double cln  = aerodynamics->get_cln();
    double cx   = aerodynamics->get_cx();
    double cz   = aerodynamics->get_cz();

    enum TVC::TVC_TYPE mtvc = tvc->get_mtvc();
    arma::vec3 FPB          = tvc->get_FPB();
    arma::vec3 FMPB         = tvc->get_FMPB();
    /*************************************************************/

    //total non-gravitational forces
    FAPB = pdynmc * refa * arma::vec({cx, cy, cz});

    //aerodynamic loss
    FAP = pdynmc * refa * arma::vec({cx, cy, cz});

    //aerodynamic moment
    FMB = pdynmc * refa * refd * arma::vec({cll, clm, cln});

    //adding thrust modified by TVC or otherwise just plain thrust if mprop > 0
    if(mtvc != TVC::NO_TVC){
        FAPB += FPB;
        FMB += FMPB;
    }else if(thrust_type != Propulsion::NO_THRUST){
        FAPB(0) += thrust;
    }

    //adding force components from RCS
    //if(rcs_enabled && (rcs_mode == ALL_GEODETIC_EULUR_ANGLE_CONTROL || rcs_mode = THRUST_VECTOR_DIRECTION_AND_ROLL_ANGLE_CONTROL))
        //FAPB=FAPB+FARCS;

    //adding moment components from RCS
    if(rcs_enabled && rcs_mode != RCS::GEODETIC_YAW_ANGLE_CONTROL){
        FMB += FMRCS;
    }
}

arma::vec Forces::get_FAPB() { return FAPB; }
arma::vec Forces::get_FAP() { return FAP; }
arma::vec Forces::get_FMB() { return FMB; }
