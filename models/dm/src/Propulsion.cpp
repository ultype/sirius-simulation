#include "integrate.hh"

#include "Propulsion.hh"
#include "sim_services/include/simtime.h"

Propulsion::Propulsion()
    :   MATRIX_INIT(IBBB, 3, 3),
        VECTOR_INIT(xcg, 3),
        VECTOR_INIT(xcg_0, 3),
        VECTOR_INIT(xcg_1, 3),
        MATRIX_INIT(I_S2_E1, 3, 3),
        MATRIX_INIT(I_S2_E1_0, 3, 3),
        MATRIX_INIT(I_S2_E1_1, 3, 3),
        MATRIX_INIT(I_S2_E2, 3, 3),
        MATRIX_INIT(I_S2_E2_0, 3, 3),
        MATRIX_INIT(I_S2_E2_1, 3, 3),
        MATRIX_INIT(I_S2_E3, 3, 3),
        MATRIX_INIT(I_S2_E3_0, 3, 3),
        MATRIX_INIT(I_S2_E3_1, 3, 3),
        MATRIX_INIT(I_S2_E4, 3, 3),
        MATRIX_INIT(I_S2_E4_0, 3, 3),
        MATRIX_INIT(I_S2_E4_1, 3, 3),
        VECTOR_INIT(structure_XCG, 3) {
    this->default_data();
}

Propulsion::Propulsion(const Propulsion& other)
    :   MATRIX_INIT(IBBB, 3, 3),
        VECTOR_INIT(xcg, 3),
        VECTOR_INIT(xcg_0, 3),
        VECTOR_INIT(xcg_1, 3),
        MATRIX_INIT(I_S2_E1, 3, 3),
        MATRIX_INIT(I_S2_E1_0, 3, 3),
        MATRIX_INIT(I_S2_E1_1, 3, 3),
        MATRIX_INIT(I_S2_E2, 3, 3),
        MATRIX_INIT(I_S2_E2_0, 3, 3),
        MATRIX_INIT(I_S2_E2_1, 3, 3),
        MATRIX_INIT(I_S2_E3, 3, 3),
        MATRIX_INIT(I_S2_E3_0, 3, 3),
        MATRIX_INIT(I_S2_E3_1, 3, 3),
        MATRIX_INIT(I_S2_E4, 3, 3),
        MATRIX_INIT(I_S2_E4_0, 3, 3),
        MATRIX_INIT(I_S2_E4_1, 3, 3),
        VECTOR_INIT(structure_XCG, 3)  {
    this->default_data();

    /* Constants */
    this->xcg_0 = other.xcg_0;
    this->xcg_1 = other.xcg_1;
    this->moi_roll_0 = other.moi_roll_0;
    this->moi_roll_1 = other.moi_roll_1;
    this->moi_pitch_0 = other.moi_pitch_0;
    this->moi_pitch_1 = other.moi_pitch_1;
    this->moi_yaw_0 = other.moi_yaw_0;
    this->moi_yaw_1 = other.moi_yaw_1;
    this->fuel_flow_rate = other.fuel_flow_rate;
    this->spi = other.spi;

    this->aexit = other.aexit;
    this->payload = other.payload;

    this->vmass0 = other.vmass0;
    this->fmass0 = other.fmass0;

    /* State */
    this->thrust_state = other.thrust_state;

    /* Propagative Stats */
    this->fmasse = other.fmasse;
    this->fmassed = other.fmassed;
    this->thrust_delta_v = other.thrust_delta_v;

    /* Generating Outputs */
    this->fmassr = other.fmassr;
    this->thrust = other.thrust;
    this->vmass = other.vmass;

    this->xcg = other.xcg;
    this->IBBB = other.IBBB;
}

Propulsion& Propulsion::operator=(const Propulsion& other) {
    if (&other == this)
        return *this;

    /* Constants */
    this->xcg_0 = other.xcg_0;
    this->xcg_1 = other.xcg_1;
    this->moi_roll_0 = other.moi_roll_0;
    this->moi_roll_1 = other.moi_roll_1;
    this->moi_pitch_0 = other.moi_pitch_0;
    this->moi_pitch_1 = other.moi_pitch_1;
    this->moi_yaw_0 = other.moi_yaw_0;
    this->moi_yaw_1 = other.moi_yaw_1;
    this->fuel_flow_rate = other.fuel_flow_rate;
    this->spi = other.spi;

    this->aexit = other.aexit;
    this->payload = other.payload;

    this->vmass0 = other.vmass0;
    this->fmass0 = other.fmass0;

    /* State */
    this->thrust_state = other.thrust_state;

    /* Propagative Stats */
    this->fmasse = other.fmasse;
    this->fmassed = other.fmassed;
    this->thrust_delta_v = other.thrust_delta_v;

    /* Generating Outputs */
    this->fmassr = other.fmassr;
    this->thrust = other.thrust;
    this->vmass = other.vmass;

    this->xcg = other.xcg;
    this->IBBB = other.IBBB;

    return *this;
}

void Propulsion::initialize() {
    this->IBBB = calculate_IBBB();
    vmass = payload + faring_mass + S2_structure_mass + S2_propellant_mass + S2_remaining_fuel_mass
            + S3_structure_mass + S3_propellant_mass + S3_remaining_fuel_mass
            - S2_fmasse;
    oxidizer_mass = fmass0 * 0.8;
    fuel_mass = fmass0 * 0.2;
    S2_timer = 0.0;
    S3_timer = 0.0;
}

void Propulsion::default_data() {
    this->fmassed = 0;
    this->thrust = 0;
    this->fmasse = 0;

    this->payload = 0;
}

void Propulsion::set_no_thrust() {
    this->thrust_state = NO_THRUST;
}

void Propulsion::set_input_thrust(double xcg0, double xcg1,
                                  double moi_roll0, double moi_roll1,
                                  double moi_pitch0, double moi_pitch1,
                                  double moi_yaw0, double moi_yaw1,
                                  double spi, double fuel_flow_rate) {
    this->thrust_state = INPUT_THRUST;

    this->fmasse = 0;

    this->xcg_0(0)          = xcg0;
    this->xcg_1(0)          = xcg1;
    this->moi_roll_0     = moi_roll0;
    this->moi_roll_1     = moi_roll1;
    this->moi_pitch_0    = moi_pitch0;
    this->moi_pitch_1    = moi_pitch1;
    this->moi_yaw_0    = moi_yaw0;
    this->moi_yaw_1    = moi_yaw1;
    this->fuel_flow_rate = fuel_flow_rate;
    this->spi            = spi;
}

void Propulsion::get_input_file_var(double xcg0, double xcg1,
                                    double moi_roll0, double moi_roll1,
                                    double moi_pitch0, double moi_pitch1,
                                    double moi_yaw0, double moi_yaw1,
                                    double spi, double fuel_flow_rate) {
    this->xcg_0(0)          = xcg0;
    this->xcg_1(0)          = xcg1;
    this->moi_roll_0     = moi_roll0;
    this->moi_roll_1     = moi_roll1;
    this->moi_pitch_0    = moi_pitch0;
    this->moi_pitch_1    = moi_pitch1;
    this->moi_yaw_0    = moi_yaw0;
    this->moi_yaw_1    = moi_yaw1;
    this->fuel_flow_rate = fuel_flow_rate;
    this->spi            = spi;
    this->fmasse = 0;
}

void Propulsion::engine_ignition() {
    this->thrust_state = INPUT_THRUST;
}
void Propulsion::set_ignition_time() {
    this->ignition_time = get_rettime();
}

void Propulsion::set_ltg_thrust() {
    this->thrust_state = LTG_THRUST;
}
void Propulsion::set_HOT_STAGE() {
    this->thrust_state = HOT_STAGE;
}
void Propulsion::set_stage_2() { this->stage = STAGE_2; }
void Propulsion::set_stage_3() { this->stage = STAGE_3; }
void Propulsion::set_faring_sep() { this->stage = FARING_SEP; }

void Propulsion::set_S2_structure_mass(double in) { S2_structure_mass = in; }
void Propulsion::set_S2_propellant_mass(double in) { S2_propellant_mass = in; }
void Propulsion::set_S2_spi(double in) { S2_spi = in; }
void Propulsion::set_S2_remaining_fuel_mass(double in) { S2_remaining_fuel_mass = in; }
void Propulsion::set_S3_structure_mass(double in) { S3_structure_mass = in; }
void Propulsion::set_S3_propellant_mass(double in) { S3_propellant_mass = in; }
void Propulsion::set_S3_spi(double in) { S3_spi = in; }
void Propulsion::set_S3_remaining_fuel_mass(double in) { S3_remaining_fuel_mass = in; }
void Propulsion::set_faring_mass(double in) { faring_mass = in; }

void Propulsion::propagate(double int_step) {
    double psl(101300);  // chamber pressure - Pa
    arma::mat33 IBBB0;
    arma::mat33 IBBB1;

    double press = grab_press();

    // no thrusting
    switch (this->thrust_state) {
        case NO_THRUST:
            thrust = 0;
            fuel_flow_rate = 0.0;
            switch (this->stage) {
                case STAGE_2:
                    mass_ratio = S2_fmasse / fmass0;
                    this->xcg = calculate_xcg();
                    this->IBBB = calculate_IBBB();
                    vmass = payload + faring_mass + S2_structure_mass + S2_propellant_mass + S2_remaining_fuel_mass
                            + S3_structure_mass + S3_propellant_mass + S3_remaining_fuel_mass
                            - S2_fmasse;
                break;

                case FARING_SEP:
                    mass_ratio = S3_fmasse / fmass0;
                    this->xcg = calculate_xcg();
                    this->IBBB = calculate_IBBB();
                    vmass = payload + S3_structure_mass + S3_propellant_mass + S3_remaining_fuel_mass
                            - S3_fmasse;
                break;

                case STAGE_3:
                    mass_ratio = S3_fmasse / fmass0;
                    this->xcg = calculate_xcg();
                    this->IBBB = calculate_IBBB();
                    vmass = payload + S3_structure_mass + S3_propellant_mass + S3_remaining_fuel_mass
                            - S3_fmasse;
                break;
            }
            break;
        case INPUT_THRUST:
            switch (this->stage) {
                case STAGE_2:
                    thrust = proptable.look_up("S2_time_vs_thrust", S2_timer, 0) * AGRAV + (- press) * this->aexit;
                    fuel_flow_rate = proptable.look_up("S2_time_vs_thrust", S2_timer, 0) / S2_spi;
                    // S2_fmasse += fuel_flow_rate * int_step;
                    fuel_expend_integrator(int_step, 2);
                    mass_ratio = S2_fmasse / fmass0;
                    this->xcg = calculate_xcg();
                    this->IBBB = calculate_IBBB();
                    vmass = payload + faring_mass + S2_structure_mass + S2_propellant_mass + S2_remaining_fuel_mass
                            + S3_structure_mass + S3_propellant_mass + S3_remaining_fuel_mass
                            - S2_fmasse;
                    propagate_thrust_delta_v(int_step, S2_spi, fuel_flow_rate, vmass);
                    if (mass_ratio >= 1.0 || fuel_flow_rate < 0.0) {
                        this->thrust_state = NO_THRUST;
                    }
                    S2_timer = S2_timer + int_step;
                break;

                case FARING_SEP:
                    thrust = proptable.look_up("S3_time_vs_thrust", S3_timer, 0) * AGRAV + (- press) * this->aexit;
                    fuel_flow_rate = proptable.look_up("S3_time_vs_thrust", S3_timer, 0) / S3_spi;
                    // S3_fmasse += fuel_flow_rate * int_step;
                    fuel_expend_integrator(int_step, 3);
                    mass_ratio = S3_fmasse / fmass0;
                    this->xcg = calculate_xcg();
                    this->IBBB = calculate_IBBB();
                    vmass = payload + S3_structure_mass + S3_propellant_mass + S3_remaining_fuel_mass
                            - S3_fmasse;
                    propagate_thrust_delta_v(int_step, S3_spi, fuel_flow_rate, vmass);
                    if (mass_ratio >= 1.0 || fuel_flow_rate < 0.0) {
                        this->thrust_state = NO_THRUST;
                    }
                    S3_timer = S3_timer + int_step;
                break;

                case STAGE_3:
                    thrust = proptable.look_up("S3_time_vs_thrust", S3_timer, 0) * AGRAV + (- press) * this->aexit;
                    fuel_flow_rate = proptable.look_up("S3_time_vs_thrust", S3_timer, 0) / S3_spi;
                    // S3_fmasse += fuel_flow_rate * int_step;
                    fuel_expend_integrator(int_step, 3);
                    mass_ratio = S3_fmasse / fmass0;
                    this->xcg = calculate_xcg();
                    this->IBBB = calculate_IBBB();
                    vmass = payload + S3_structure_mass + S3_propellant_mass + S3_remaining_fuel_mass + faring_mass
                            - S3_fmasse;
                    propagate_thrust_delta_v(int_step, S3_spi, fuel_flow_rate, vmass);
                    if (mass_ratio >= 1.0 || fuel_flow_rate < 0.0) {
                        this->thrust_state = NO_THRUST;
                    }
                    S3_timer = S3_timer + int_step;
                break;
            }

            break;

        case HOT_STAGE :
                vmass = payload + faring_mass + S2_structure_mass + S2_propellant_mass + S2_remaining_fuel_mass
                            + S3_structure_mass + S3_propellant_mass + S3_remaining_fuel_mass;
                thrust = proptable.look_up("S2_time_vs_thrust", S2_timer, 0) * AGRAV + (- press) * this->aexit;
                fuel_flow_rate = proptable.look_up("S2_time_vs_thrust", S2_timer, 0) / S2_spi;
                double fuel_flow_rate_hs = proptable.look_up("S3_time_vs_thrust", S3_timer, 0) / S3_spi;
                if (mass_ratio <= 1.0) {
                    fuel_expend_integrator(int_step, 2);
                } else {
                    fuel_flow_rate = 0.0;
                    thrust = 0.0;
                }
                // S2_fmasse += fuel_flow_rate * int_step;

                // S3_fmasse += fuel_flow_rate_hs * int_step;
                fuel_expend_integrator(int_step, 3);
                mass_ratio = S2_fmasse / fmass0;
                this->xcg = calculate_xcg();
                this->IBBB = calculate_IBBB();
                vmass = payload + faring_mass + S2_structure_mass + S2_propellant_mass + S2_remaining_fuel_mass
                            + S3_structure_mass + S3_propellant_mass + S3_remaining_fuel_mass
                            - S2_fmasse - S3_fmasse;
                propagate_thrust_delta_v(int_step, S2_spi, fuel_flow_rate, vmass);

                S3_timer = S3_timer + int_step;
                S2_timer = S2_timer + int_step;
            break;
    }
    fmasse = S2_fmasse + S3_fmasse;
    if (thrust < 0.0) {
        thrust = 0.0;
    }
}

void Propulsion::fuel_expend_integrator(double int_step, unsigned int flag) {
    double K1, K2, K3, K4;
    switch (flag) {
        case 2 :
            K1 = proptable.look_up("S2_time_vs_thrust", S2_timer, 0) / S2_spi;
            K2 = proptable.look_up("S2_time_vs_thrust", S2_timer + 0.5 * int_step, 0) / S2_spi;
            K3 = proptable.look_up("S2_time_vs_thrust", S2_timer + 0.5 * int_step, 0) / S2_spi;
            K4 = proptable.look_up("S2_time_vs_thrust", S2_timer + int_step, 0) / S2_spi;

            S2_fmasse = S2_fmasse + (int_step / 6.0) * (K1 + 2.0 * K2 + 2.0 * K3 + K4);
        break;

        case 3 :
            K1 = proptable.look_up("S3_time_vs_thrust", S3_timer, 0) / S3_spi;
            K2 = proptable.look_up("S3_time_vs_thrust", S3_timer + 0.5 * int_step, 0) / S3_spi;
            K3 = proptable.look_up("S3_time_vs_thrust", S3_timer + 0.5 * int_step, 0) / S3_spi;
            K4 = proptable.look_up("S3_time_vs_thrust", S3_timer + int_step, 0) / S3_spi;

            S3_fmasse = S3_fmasse + (int_step / 6.0) * (K1 + 2.0 * K2 + 2.0 * K3 + K4);
        break;
    }
}

void Propulsion::propagate_thrust_delta_v(double int_step, double spi, double fuel_flow_rate, double vmass) {
    this->thrust_delta_v += spi * AGRAV * (fuel_flow_rate / vmass) * int_step;
}

void Propulsion::propagate_fmasse(double int_step) {
    // INTEGRATE(fmasse, (this->thrust - (- press) * this->aexit) / (this->spi * AGRAV));  // thrust/(spi*AGRAV)
    // INTEGRATE(fmasse, (this->thrust - (psl - press) * this->aexit) / (this->spi * 9.8));
    // fmasse += ((this->thrust - (- press) * this->aexit) / (this->spi * AGRAV)) * int_step;
    fmasse += fuel_flow_rate * int_step;
}
double Propulsion::calculate_thrust(double press) {
    // return this->spi * this->fuel_flow_rate * AGRAV + (- press) * this->aexit;
    // return this->spi * this->fuel_flow_rate * 9.8 + (psl - press) * this->aexit;
    return proptable.look_up("time_vs_thrust", get_rettime(), 0) * AGRAV + (- press) * this->aexit;
}

double Propulsion::calculate_fmassr() {
    return this->fmass0 - this->fmasse;
}

arma::vec3 Propulsion::calculate_xcg() {
    arma::vec3 SLOSH_CG = grab_SLOSH_CG();
    double slosh_mass = grab_slosh_mass();
    arma::vec3 e1_XCG = grab_e1_XCG();
    arma::vec3 e2_XCG = grab_e2_XCG();
    arma::vec3 e3_XCG = grab_e3_XCG();
    arma::vec3 e4_XCG = grab_e4_XCG();

    if (CG_OFFSET) {
        structure_XCG = ((slosh_mass * SLOSH_CG) + (vmass - slosh_mass - S2_E1_mass - S2_E2_mass - S2_E3_mass - S2_E4_mass) * (-(xcg_0 + (xcg_1 - xcg_0) * mass_ratio))) / (vmass - S2_E1_mass - S2_E2_mass - S2_E3_mass - S2_E4_mass);
        return ((slosh_mass * SLOSH_CG) + ((vmass - slosh_mass - S2_E1_mass - S2_E2_mass - S2_E3_mass - S2_E4_mass) * (-(xcg_0 + (xcg_1 - xcg_0) * mass_ratio)))
                    + (S2_E1_mass * e1_XCG) + (S2_E2_mass * e2_XCG) + (S2_E3_mass * e3_XCG) + (S2_E4_mass * e4_XCG)) / vmass;
    }
    structure_XCG = -(xcg_0 + (xcg_1 - xcg_0) * mass_ratio);
    return -(xcg_0 + (xcg_1 - xcg_0) * mass_ratio);
}

arma::mat33 Propulsion::calculate_IBBB() {
    return get_IBBB0() + (get_IBBB1() - get_IBBB0()) * mass_ratio;
}

arma::mat33 Propulsion::get_IBBB0() {
    arma::mat33 IBBB0(arma::fill::zeros);
    IBBB0(0, 0) = moi_roll_0;
    IBBB0(1, 1) = moi_pitch_0;
    IBBB0(2, 2) = moi_yaw_0;

    return IBBB0;
}

arma::mat33 Propulsion::get_IBBB1() {
    arma::mat33 IBBB1(arma::fill::zeros);
    IBBB1(0, 0) = moi_roll_1;
    IBBB1(1, 1) = moi_pitch_1;
    IBBB1(2, 2) = moi_yaw_1;

    return IBBB1;
}

void Propulsion::calcuate_engine_variable() {
    S2_E1_xcg = S2_E1_xcg_0 + (S2_E1_xcg_1 - S2_E1_xcg_0) * mass_ratio;
    S2_E2_xcg = S2_E2_xcg_0 + (S2_E2_xcg_1 - S2_E2_xcg_0) * mass_ratio;
    S2_E3_xcg = S2_E3_xcg_0 + (S2_E3_xcg_1 - S2_E3_xcg_0) * mass_ratio;
    S2_E4_xcg = S2_E4_xcg_0 + (S2_E4_xcg_1 - S2_E4_xcg_0) * mass_ratio;
    I_S2_E1 = I_S2_E1_0 + (I_S2_E1_1 - I_S2_E1_0) * mass_ratio;
    I_S2_E2 = I_S2_E2_0 + (I_S2_E2_1 - I_S2_E2_0) * mass_ratio;
    I_S2_E3 = I_S2_E3_0 + (I_S2_E3_1 - I_S2_E3_0) * mass_ratio;
    I_S2_E4 = I_S2_E4_0 + (I_S2_E4_1 - I_S2_E4_0) * mass_ratio;
    fuel_mass = (1. - mass_ratio) * (S2_E1_fuel_mass + S2_E2_fuel_mass + S2_E3_fuel_mass + S2_E4_fuel_mass);
    S2_E1_mass = S2_E1_mass_1 + fuel_mass * 0.25;
    S2_E2_mass = S2_E2_mass_1 + fuel_mass * 0.25;
    S2_E3_mass = S2_E3_mass_1 + fuel_mass * 0.25;
    S2_E4_mass = S2_E4_mass_1 + fuel_mass * 0.25;
}

void Propulsion::load_proptable(const char* filename) {
    proptable = Datadeck(filename);
}

double Propulsion::get_S2_E1_xcg() { return S2_E1_xcg; }
double Propulsion::get_S2_E2_xcg() { return S2_E2_xcg; }
double Propulsion::get_S2_E3_xcg() { return S2_E3_xcg; }
double Propulsion::get_S2_E4_xcg() { return S2_E4_xcg; }
double Propulsion::get_S2_E1_mass() { return S2_E1_mass; }
double Propulsion::get_S2_E2_mass() { return S2_E2_mass; }
double Propulsion::get_S2_E3_mass() { return S2_E3_mass; }
double Propulsion::get_S2_E4_mass() { return S2_E4_mass; }
arma::mat33 Propulsion::get_I_S2_E1() { return I_S2_E1; }
arma::mat33 Propulsion::get_I_S2_E2() { return I_S2_E2; }
arma::mat33 Propulsion::get_I_S2_E3() { return I_S2_E3; }
arma::mat33 Propulsion::get_I_S2_E4() { return I_S2_E4; }
arma::vec3 Propulsion::get_structure_XCG() { return structure_XCG; }

void Propulsion::set_vmass0(double in) { vmass0 = in; }
void Propulsion::set_fmass0(double in) { fmass0 = in; }
void Propulsion::set_aexit(double in) { aexit = in; }
void Propulsion::set_payload(double in) { payload = in; }
void Propulsion::set_CG_OFFSET(unsigned int in) { CG_OFFSET = in; }
void Propulsion::set_TWD(unsigned int in) { TWD = in; }
void Propulsion::set_S2_E1_VARIABLE(double in1, double in2, double in3, double in4, double in5, double in6, double in7
                                    , double in8, double in9, double in10) {
    S2_E1_xcg_0 = in1;
    S2_E1_xcg_1 = in2;
    S2_E1_roll_0 = in3;
    S2_E1_roll_1 = in4;
    S2_E1_pitch_0 = in5;
    S2_E1_pitch_1 = in6;
    S2_E1_yaw_0 = in7;
    S2_E1_yaw_1 = in8;
    S2_E1_mass_0 = in9;
    S2_E1_mass_1 = in10;

    I_S2_E1_0(0, 0) = S2_E1_roll_0;
    I_S2_E1_0(1, 1) = S2_E1_pitch_0;
    I_S2_E1_0(2, 2) = S2_E1_yaw_0;

    I_S2_E1_1(0, 0) = S2_E1_roll_1;
    I_S2_E1_1(1, 1) = S2_E1_pitch_1;
    I_S2_E1_1(2, 2) = S2_E1_yaw_1;

    S2_E1_fuel_mass = S2_E1_mass_0 - S2_E1_mass_1;
}

void Propulsion::set_S2_E2_VARIABLE(double in1, double in2, double in3, double in4, double in5, double in6, double in7
                                    , double in8, double in9, double in10) {
    S2_E2_xcg_0 = in1;
    S2_E2_xcg_1 = in2;
    S2_E2_roll_0 = in3;
    S2_E2_roll_1 = in4;
    S2_E2_pitch_0 = in5;
    S2_E2_pitch_1 = in6;
    S2_E2_yaw_0 = in7;
    S2_E2_yaw_1 = in8;
    S2_E2_mass_0 = in9;
    S2_E2_mass_1 = in10;

    I_S2_E2_0(0, 0) = S2_E2_roll_0;
    I_S2_E2_0(1, 1) = S2_E2_pitch_0;
    I_S2_E2_0(2, 2) = S2_E2_yaw_0;

    I_S2_E2_1(0, 0) = S2_E2_roll_1;
    I_S2_E2_1(1, 1) = S2_E2_pitch_1;
    I_S2_E2_1(2, 2) = S2_E2_yaw_1;

    S2_E2_fuel_mass = S2_E2_mass_0 - S2_E2_mass_1;
}

void Propulsion::set_S2_E3_VARIABLE(double in1, double in2, double in3, double in4, double in5, double in6, double in7
                                    , double in8, double in9, double in10) {
    S2_E3_xcg_0 = in1;
    S2_E3_xcg_1 = in2;
    S2_E3_roll_0 = in3;
    S2_E3_roll_1 = in4;
    S2_E3_pitch_0 = in5;
    S2_E3_pitch_1 = in6;
    S2_E3_yaw_0 = in7;
    S2_E3_yaw_1 = in8;
    S2_E3_mass_0 = in9;
    S2_E3_mass_1 = in10;

    I_S2_E3_0(0, 0) = S2_E3_roll_0;
    I_S2_E3_0(1, 1) = S2_E3_pitch_0;
    I_S2_E3_0(2, 2) = S2_E3_yaw_0;

    I_S2_E3_1(0, 0) = S2_E3_roll_1;
    I_S2_E3_1(1, 1) = S2_E3_pitch_1;
    I_S2_E3_1(2, 2) = S2_E3_yaw_1;

    S2_E3_fuel_mass = S2_E3_mass_0 - S2_E3_mass_1;
}

void Propulsion::set_S2_E4_VARIABLE(double in1, double in2, double in3, double in4, double in5, double in6, double in7
                                    , double in8, double in9, double in10) {
    S2_E4_xcg_0 = in1;
    S2_E4_xcg_1 = in2;
    S2_E4_roll_0 = in3;
    S2_E4_roll_1 = in4;
    S2_E4_pitch_0 = in5;
    S2_E4_pitch_1 = in6;
    S2_E4_yaw_0 = in7;
    S2_E4_yaw_1 = in8;
    S2_E4_mass_0 = in9;
    S2_E4_mass_1 = in10;

    I_S2_E4_0(0, 0) = S2_E4_roll_0;
    I_S2_E4_0(1, 1) = S2_E4_pitch_0;
    I_S2_E4_0(2, 2) = S2_E4_yaw_0;

    I_S2_E4_1(0, 0) = S2_E4_roll_1;
    I_S2_E4_1(1, 1) = S2_E4_pitch_1;
    I_S2_E4_1(2, 2) = S2_E4_yaw_1;

    S2_E4_fuel_mass = S2_E4_mass_0 - S2_E4_mass_1;
}

int Propulsion::get_mprop() { return static_cast<int>(this->thrust_state); /*XXX work around*/ }
enum Propulsion::THRUST_TYPE Propulsion::get_thrust_state() { return this->thrust_state; }
double Propulsion::get_vmass() { return vmass; }
arma::vec3 Propulsion::get_xcg() { return xcg; }
arma::vec3 Propulsion::get_xcg_0() { return xcg_0; }
double Propulsion::get_thrust() { return thrust; }
double Propulsion::get_fmassr() { return fmassr; }
double Propulsion::get_oxidizer_mass() { return oxidizer_mass; }

arma::mat33 Propulsion::get_IBBB() { return IBBB; }
