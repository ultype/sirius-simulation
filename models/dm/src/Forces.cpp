#include "Force.hh"
#include "sim_services/include/simtime.h"
#include "aux.hh"
#include <cmath>
#define NRANSI
#include "nrutil.h"
#define MAXITS 20000000
#define ALF 1.0e-4
#define EPS 1.0e-7  // 1.0e-7
#define TOLF 1.0e-8  // 1.0e-4
#define TOLX 1.0e-9
#define STPMX 100.0
#define TOLMIN 1.0e-10  // 1.0e-6
#define FREERETURN do { free_dvector(fvec, 1, n);\
                        free_dvector(xold, 1, n);\
                        free_dvector(w, 1, n);\
                        free_dvector(t, 1, n);\
                        free_dvector(s, 1, n);\
                        free_dmatrix(r, 1, n, 1, n);\
                        free_dmatrix(qt, 1, n, 1, n);\
                        free_dvector(p, 1, n);\
                        free_dvector(g, 1, n);\
                        free_dvector(fvcold, 1, n);\
                        free_dvector(d, 1, n);\
                        free_dvector(c, 1, n);\
                        return;\
} while (0);

double *fvec;
int nn;

Forces::Forces(Propulsion& prop, TVC& tvc)
    :   propulsion(&prop), tvc(&tvc),
        VECTOR_INIT(FAPB, 3),
        VECTOR_INIT(FAP, 3),
        VECTOR_INIT(FMB, 3),
        VECTOR_INIT(FMAB, 3),
        VECTOR_INIT(Q_G, 8),
        VECTOR_INIT(Q_Aero, 6),
        VECTOR_INIT(rhoC_1, 3),
        MATRIX_INIT(I1, 3, 3),
        VECTOR_INIT(ddrP_1, 3),
        VECTOR_INIT(ddang_1, 3),
        VECTOR_INIT(dang_1, 3),
        VECTOR_INIT(ddrhoC_1, 3),
        VECTOR_INIT(p_b1_ga, 3),
        VECTOR_INIT(p_b1_be, 3),
        VECTOR_INIT(f, 12),
        VECTOR_INIT(f_slosh, 8),
        VECTOR_INIT(gamma_b1_q1, 3),
        VECTOR_INIT(gamma_b1_q2, 3),
        VECTOR_INIT(gamma_b1_q3, 3),
        VECTOR_INIT(gamma_b1_q4, 3),
        VECTOR_INIT(gamma_b1_q5, 3),
        VECTOR_INIT(gamma_b1_q6, 3),
        VECTOR_INIT(beta_b1_q1, 3),
        VECTOR_INIT(beta_b1_q2, 3),
        VECTOR_INIT(beta_b1_q3, 3),
        VECTOR_INIT(beta_b1_q4, 3),
        VECTOR_INIT(beta_b1_q5, 3),
        VECTOR_INIT(beta_b1_q6, 3),
        VECTOR_INIT(beta_slosh_q4, 3),
        VECTOR_INIT(beta_slosh_q5, 3),
        VECTOR_INIT(beta_slosh_q6, 3),
        VECTOR_INIT(beta_slosh_q7, 3),
        VECTOR_INIT(beta_slosh_q8, 3),
        VECTOR_INIT(ddang_slosh, 3),
        VECTOR_INIT(ddrP_slosh, 3),
        VECTOR_INIT(ddrhoC_slosh, 3),
        VECTOR_INIT(rhoC_slosh, 3),
        VECTOR_INIT(r_slosh, 3),
        VECTOR_INIT(Q_G_slosh, 8),
        MATRIX_INIT(TBSLOSH_I, 3, 3),
        VECTOR_INIT(dang_slosh1, 3),
        VECTOR_INIT(dang_slosh2, 3),
        VECTOR_INIT(omega_slosh1_B, 3),
        VECTOR_INIT(omega_slosh2_slosh1, 3),
        MATRIX_INIT(TSLOSHB1_B, 3, 3),
        MATRIX_INIT(TSLOSHB2_SLOSHB1, 3, 3),
        VECTOR_INIT(SLOSH_CG, 3),
        VECTOR_INIT(beta_S2_e1_q4, 3),
        VECTOR_INIT(beta_S2_e1_q5, 3),
        VECTOR_INIT(beta_S2_e1_q6, 3),
        VECTOR_INIT(beta_S2_e1_q_theta, 3),
        VECTOR_INIT(beta_S2_e2_q4, 3),
        VECTOR_INIT(beta_S2_e2_q5, 3),
        VECTOR_INIT(beta_S2_e2_q6, 3),
        VECTOR_INIT(beta_S2_e2_q_psi, 3),
        VECTOR_INIT(beta_S2_e3_q4, 3),
        VECTOR_INIT(beta_S2_e3_q5, 3),
        VECTOR_INIT(beta_S2_e3_q6, 3),
        VECTOR_INIT(beta_S2_e3_q_theta, 3),
        VECTOR_INIT(beta_S2_e4_q4, 3),
        VECTOR_INIT(beta_S2_e4_q5, 3),
        VECTOR_INIT(beta_S2_e4_q6, 3),
        VECTOR_INIT(beta_S2_e4_q_psi, 3),
        VECTOR_INIT(ddrP_e1, 3),
        VECTOR_INIT(ddrP_e2, 3),
        VECTOR_INIT(ddrP_e3, 3),
        VECTOR_INIT(ddrP_e4, 3),
        VECTOR_INIT(ddrhoC_e1, 3),
        VECTOR_INIT(ddrhoC_e2, 3),
        VECTOR_INIT(ddrhoC_e3, 3),
        VECTOR_INIT(ddrhoC_e4, 3),
        VECTOR_INIT(Q_G_S2_E, 10),
        VECTOR_INIT(f_S2_TWD, 10),
        MATRIX_INIT(TE1_B, 3, 3),
        MATRIX_INIT(TE2_B, 3, 3),
        MATRIX_INIT(TE3_B, 3, 3),
        MATRIX_INIT(TE4_B, 3, 3),
        MATRIX_INIT(TE1_I, 3, 3),
        MATRIX_INIT(TE2_I, 3, 3),
        MATRIX_INIT(TE3_I, 3, 3),
        MATRIX_INIT(TE4_I, 3, 3),
        VECTOR_INIT(omega_e1_B, 3),
        VECTOR_INIT(omega_e2_B, 3),
        VECTOR_INIT(omega_e3_B, 3),
        VECTOR_INIT(omega_e4_B, 3),
        VECTOR_INIT(domega_e1, 3),
        VECTOR_INIT(domega_e2, 3),
        VECTOR_INIT(domega_e3, 3),
        VECTOR_INIT(domega_e4, 3),
        VECTOR_INIT(e1_d, 3),
        VECTOR_INIT(e2_d, 3),
        VECTOR_INIT(e3_d, 3),
        VECTOR_INIT(e4_d, 3),
        VECTOR_INIT(dang_e1, 3),
        VECTOR_INIT(dang_e2, 3),
        VECTOR_INIT(dang_e3, 3),
        VECTOR_INIT(dang_e4, 3),
        VECTOR_INIT(rhoC_e1, 3),
        VECTOR_INIT(rhoC_e2, 3),
        VECTOR_INIT(rhoC_e3, 3),
        VECTOR_INIT(rhoC_e4, 3),
        MATRIX_INIT(I_E1, 3, 3),
        MATRIX_INIT(I_E2, 3, 3),
        MATRIX_INIT(I_E3, 3, 3),
        MATRIX_INIT(I_E4, 3, 3),
        VECTOR_INIT(e1_XCG, 3),
        VECTOR_INIT(e2_XCG, 3),
        VECTOR_INIT(e3_XCG, 3),
        VECTOR_INIT(e4_XCG, 3),
        VECTOR_INIT(ddrP_sloshB, 3) {
    this->default_data();
}
Forces::Forces(const Forces& other)
    :   propulsion(other.propulsion), tvc(other.tvc),
        VECTOR_INIT(FAPB, 3),
        VECTOR_INIT(FAP, 3),
        VECTOR_INIT(FMB, 3),
        VECTOR_INIT(FMAB, 3),
        VECTOR_INIT(Q_G, 8),
        VECTOR_INIT(Q_Aero, 6),
        VECTOR_INIT(rhoC_1, 3),
        MATRIX_INIT(I1, 3, 3),
        VECTOR_INIT(ddrP_1, 3),
        VECTOR_INIT(ddang_1, 3),
        VECTOR_INIT(dang_1, 3),
        VECTOR_INIT(ddrhoC_1, 3),
        VECTOR_INIT(p_b1_ga, 3),
        VECTOR_INIT(p_b1_be, 3),
        VECTOR_INIT(f, 12),
        VECTOR_INIT(f_slosh, 8),
        VECTOR_INIT(gamma_b1_q1, 3),
        VECTOR_INIT(gamma_b1_q2, 3),
        VECTOR_INIT(gamma_b1_q3, 3),
        VECTOR_INIT(gamma_b1_q4, 3),
        VECTOR_INIT(gamma_b1_q5, 3),
        VECTOR_INIT(gamma_b1_q6, 3),
        VECTOR_INIT(beta_b1_q1, 3),
        VECTOR_INIT(beta_b1_q2, 3),
        VECTOR_INIT(beta_b1_q3, 3),
        VECTOR_INIT(beta_b1_q4, 3),
        VECTOR_INIT(beta_b1_q5, 3),
        VECTOR_INIT(beta_b1_q6, 3),
        VECTOR_INIT(beta_slosh_q4, 3),
        VECTOR_INIT(beta_slosh_q5, 3),
        VECTOR_INIT(beta_slosh_q6, 3),
        VECTOR_INIT(beta_slosh_q7, 3),
        VECTOR_INIT(beta_slosh_q8, 3),
        VECTOR_INIT(ddang_slosh, 3),
        VECTOR_INIT(ddrP_slosh, 3),
        VECTOR_INIT(ddrhoC_slosh, 3),
        VECTOR_INIT(rhoC_slosh, 3),
        VECTOR_INIT(r_slosh, 3),
        VECTOR_INIT(Q_G_slosh, 8),
        MATRIX_INIT(TBSLOSH_I, 3, 3),
        VECTOR_INIT(dang_slosh1, 3),
        VECTOR_INIT(dang_slosh2, 3),
        VECTOR_INIT(omega_slosh1_B, 3),
        VECTOR_INIT(omega_slosh2_slosh1, 3),
        MATRIX_INIT(TSLOSHB1_B, 3, 3),
        MATRIX_INIT(TSLOSHB2_SLOSHB1, 3, 3),
        VECTOR_INIT(SLOSH_CG, 3),
        VECTOR_INIT(beta_S2_e1_q4, 3),
        VECTOR_INIT(beta_S2_e1_q5, 3),
        VECTOR_INIT(beta_S2_e1_q6, 3),
        VECTOR_INIT(beta_S2_e1_q_theta, 3),
        VECTOR_INIT(beta_S2_e2_q4, 3),
        VECTOR_INIT(beta_S2_e2_q5, 3),
        VECTOR_INIT(beta_S2_e2_q6, 3),
        VECTOR_INIT(beta_S2_e2_q_psi, 3),
        VECTOR_INIT(beta_S2_e3_q4, 3),
        VECTOR_INIT(beta_S2_e3_q5, 3),
        VECTOR_INIT(beta_S2_e3_q6, 3),
        VECTOR_INIT(beta_S2_e3_q_theta, 3),
        VECTOR_INIT(beta_S2_e4_q4, 3),
        VECTOR_INIT(beta_S2_e4_q5, 3),
        VECTOR_INIT(beta_S2_e4_q6, 3),
        VECTOR_INIT(beta_S2_e4_q_psi, 3),
        VECTOR_INIT(ddrP_e1, 3),
        VECTOR_INIT(ddrP_e2, 3),
        VECTOR_INIT(ddrP_e3, 3),
        VECTOR_INIT(ddrP_e4, 3),
        VECTOR_INIT(ddrhoC_e1, 3),
        VECTOR_INIT(ddrhoC_e2, 3),
        VECTOR_INIT(ddrhoC_e3, 3),
        VECTOR_INIT(ddrhoC_e4, 3),
        VECTOR_INIT(Q_G_S2_E, 10),
        VECTOR_INIT(f_S2_TWD, 10),
        MATRIX_INIT(TE1_B, 3, 3),
        MATRIX_INIT(TE2_B, 3, 3),
        MATRIX_INIT(TE3_B, 3, 3),
        MATRIX_INIT(TE4_B, 3, 3),
        MATRIX_INIT(TE1_I, 3, 3),
        MATRIX_INIT(TE2_I, 3, 3),
        MATRIX_INIT(TE3_I, 3, 3),
        MATRIX_INIT(TE4_I, 3, 3),
        VECTOR_INIT(omega_e1_B, 3),
        VECTOR_INIT(omega_e2_B, 3),
        VECTOR_INIT(omega_e3_B, 3),
        VECTOR_INIT(omega_e4_B, 3),
        VECTOR_INIT(domega_e1, 3),
        VECTOR_INIT(domega_e2, 3),
        VECTOR_INIT(domega_e3, 3),
        VECTOR_INIT(domega_e4, 3),
        VECTOR_INIT(e1_d, 3),
        VECTOR_INIT(e2_d, 3),
        VECTOR_INIT(e3_d, 3),
        VECTOR_INIT(e4_d, 3),
        VECTOR_INIT(dang_e1, 3),
        VECTOR_INIT(dang_e2, 3),
        VECTOR_INIT(dang_e3, 3),
        VECTOR_INIT(dang_e4, 3),
        VECTOR_INIT(rhoC_e1, 3),
        VECTOR_INIT(rhoC_e2, 3),
        VECTOR_INIT(rhoC_e3, 3),
        VECTOR_INIT(rhoC_e4, 3),
        MATRIX_INIT(I_E1, 3, 3),
        MATRIX_INIT(I_E2, 3, 3),
        MATRIX_INIT(I_E3, 3, 3),
        MATRIX_INIT(I_E4, 3, 3),
        VECTOR_INIT(e1_XCG, 3),
        VECTOR_INIT(e2_XCG, 3),
        VECTOR_INIT(e3_XCG, 3),
        VECTOR_INIT(e4_XCG, 3),
        VECTOR_INIT(ddrP_sloshB, 3) {
    this->default_data();

    this->FAP = other.FAP;
    this->FAPB = other.FAPB;
    this->FMB = other.FMB;
}

Forces& Forces::operator=(const Forces& other) {
    if (&other == this)
        return *this;

    this->propulsion = other.propulsion;
    this->tvc = other.tvc;

    this->FAP = other.FAP;
    this->FAPB = other.FAPB;
    this->FMB = other.FMB;

    return *this;
}

void Forces::default_data() {
}


void Forces::initialize() {
    gamma_beta();
    // Gravity_Q();
}

void Forces::set_Slosh_flag(unsigned int flag) { Slosh_flag = flag; }
void Forces::set_DOF(int ndof) { DOF = ndof ;}
void Forces::set_damping_ratio(double damping) { damping_ratio = damping; }
void Forces::set_aero_flag(unsigned int in) { Aero_flag = in; }
void Forces::set_e1_d(double in1, double in2, double in3) {
    e1_d(0) = in1;
    e1_d(1) = in2;
    e1_d(2) = in3;
}
void Forces::set_e2_d(double in1, double in2, double in3) {
    e2_d(0) = in1;
    e2_d(1) = in2;
    e2_d(2) = in3;
}
void Forces::set_e3_d(double in1, double in2, double in3) {
    e3_d(0) = in1;
    e3_d(1) = in2;
    e3_d(2) = in3;
}
void Forces::set_e4_d(double in1, double in2, double in3) {
    e4_d(0) = in1;
    e4_d(1) = in2;
    e4_d(2) = in3;
}
void Forces::collect_forces_and_propagate() {
    double *ff, *x;
    int check(0);
    ff = new double[13];
    x = new double[13];
    /*****************input from another module*******************/
    rhoC_1 = grab_structure_XCG();
    dang_1 = grab_WBIB();
    arma::mat33 TBI = grab_TBI();
    arma::vec3 FSPB = grab_FSPB();

    rhoC_1(0) = rhoC_1(0) - xp;

    if (Slosh_flag == 1) {
        double dang_slosh_theta = grab_dang_slosh_theta();
        double ang_slosh_theta = grab_ang_slosh_theta();
        double dang_slosh_psi = grab_dang_slosh_psi();
        double ang_slosh_psi = grab_ang_slosh_psi();
        /***********************************************/
        TSLOSHB1_B = TMY(ang_slosh_theta);
        TSLOSHB2_SLOSHB1 = TMZ(ang_slosh_psi);
        TBSLOSH_I = TSLOSHB2_SLOSHB1 * TSLOSHB1_B * TBI;
        /***********************************************/
        omega_slosh1_B(0) = 0.0;
        omega_slosh1_B(1) = dang_slosh_theta;
        omega_slosh1_B(2) = 0.0;

        omega_slosh2_slosh1(0) = 0.0;
        omega_slosh2_slosh1(1) = 0.0;
        omega_slosh2_slosh1(2) = dang_slosh_psi;

        double m_liq = grab_oxidizer_mass();
        h = (m_liq / 1400.0) / 0.838218;
        double hn = h - (0.516 / 1.841) * (tanh((1.841 * h) / 2. * 0.516) - (1. / sinh((1.841 * h)/ 0.516)));

        slosh_mass = (1.0 * (m_liq * ((2 * 0.516 * tanh(1.841 * h / 0.516)) / (1.841 * (1.841 * 1.841 - 1.) * h))));
        pendulum_L = 0.516 / (1.841 * tanh(1.841 * h / 0.516));

        dang_slosh1 = TSLOSHB1_B * dang_1 + omega_slosh1_B;
        dang_slosh2 = TSLOSHB2_SLOSHB1 * dang_slosh1 + omega_slosh2_slosh1;

        rhoC_slosh(0) = -pendulum_L;
        rhoC_slosh(1) = 0.0;
        rhoC_slosh(2) = 0.0;

        r_slosh(0) = (-6.436 + hn) - xp;
        r_slosh(1) = 0.0;
        r_slosh(2) = 0.0;

        r_slosh = r_slosh - rhoC_slosh;

        SLOSH_CG = r_slosh + trans(TSLOSHB2_SLOSHB1 * TSLOSHB1_B) * rhoC_slosh;
        SLOSH_CG(0) = SLOSH_CG(0) + xp;

        Wn = sqrt(((fabs(FSPB(0)) * 1.841) / 0.516) * tanh((1.841 * h) / (0.516)));
    }

    if (TWD_flag == 1) {
        double dang_e1_B = grab_dang_e1_B();
        double dang_e2_B = grab_dang_e2_B();
        double dang_e3_B = grab_dang_e3_B();
        double dang_e4_B = grab_dang_e4_B();

        double ang_e1_theta = grab_ang_e1_theta();
        double ang_e2_psi = grab_ang_e2_psi();
        double ang_e3_theta = grab_ang_e3_theta();
        double ang_e4_psi = grab_ang_e4_psi();

        double e1_rhoC = grab_e1_XCG();
        double e2_rhoC = grab_e2_XCG();
        double e3_rhoC = grab_e3_XCG();
        double e4_rhoC = grab_e4_XCG();

        ddang_e1_theta = grab_s2_act1_acc();
        ddang_e2_psi = grab_s2_act2_acc();
        ddang_e3_theta = grab_s2_act3_acc();
        ddang_e4_psi = grab_s2_act4_acc();
        /*********************************************/
        TE1_B = TMY(ang_e1_theta);
        TE2_B = TMZ(ang_e2_psi);
        TE3_B = TMY(ang_e3_theta);
        TE4_B = TMZ(ang_e4_psi);

        TE1_I = TE1_B * TBI;
        TE2_I = TE2_B * TBI;
        TE3_I = TE3_B * TBI;
        TE4_I = TE4_B * TBI;
        /*********************************************/
        omega_e1_B(0) = 0.0;
        omega_e1_B(1) = dang_e1_B;
        omega_e1_B(2) = 0.0;

        omega_e2_B(0) = 0.0;
        omega_e2_B(1) = 0.0;
        omega_e2_B(2) = dang_e2_B;

        omega_e3_B(0) = 0.0;
        omega_e3_B(1) = dang_e3_B;
        omega_e3_B(2) = 0.0;

        omega_e4_B(0) = 0.0;
        omega_e4_B(1) = 0.0;
        omega_e4_B(2) = dang_e4_B;
        /**********************************************/
        dang_e1 = TE1_B * dang_1 + omega_e1_B;
        dang_e2 = TE2_B * dang_1 + omega_e2_B;
        dang_e3 = TE3_B * dang_1 + omega_e3_B;
        dang_e4 = TE4_B * dang_1 + omega_e4_B;
        /**********************************************/
        rhoC_e1(0) = -e1_rhoC;
        rhoC_e1(1) = 0.0;
        rhoC_e1(2) = 0.0;

        rhoC_e2(0) = -e2_rhoC;
        rhoC_e2(1) = 0.0;
        rhoC_e2(2) = 0.0;

        rhoC_e3(0) = -e3_rhoC;
        rhoC_e3(1) = 0.0;
        rhoC_e3(2) = 0.0;

        rhoC_e4(0) = -e4_rhoC;
        rhoC_e4(1) = 0.0;
        rhoC_e4(2) = 0.0;
        /************************************************/
        e1_XCG = trans(TE1_B) * (rhoC_e1) + e1_d;
        e1_XCG(0) = e1_XCG(0) + xp;

        e2_XCG = trans(TE2_B) * (rhoC_e2) + e2_d;
        e2_XCG(0) = e2_XCG(0) + xp;

        e3_XCG = trans(TE3_B) * (rhoC_e3) + e3_d;
        e3_XCG(0) = e3_XCG(0) + xp;

        e4_XCG = trans(TE4_B) * (rhoC_e4) + e4_d;
        e4_XCG(0) = e4_XCG(0) + xp;
        /************************************************/
        calculate_I_E();
    }

    calculate_I1();
    gamma_beta();
    Gravity_Q();
    if (Aero_flag == 1) {
        AeroDynamics_Q();
    }

    for (int i = 0; i < 12; i++) {
        x[i + 1] = 0.0;
    }

    broydn(x, DOF, &check);

    funcv(DOF, x, ff);
    for (int i = 0; i < 3; i++) {
        ddrP_1(i) = x[i + 1];
        ddang_1(i) = x[i + 4];
    }
    if (Slosh_flag == 1) {
        ddang_slosh_theta = x[7];
        ddang_slosh_psi = x[8];
        // Q_slosh_7 = x[7];
        // Q_slosh_8 = x[8];
    }
    if (TWD_flag == 1) {
        Q_E1 = x[7];
        Q_E2 = x[8];
        Q_E3 = x[9];
        Q_E4 = x[10];
    }

    if (Slosh_flag == 1 && TWD_flag == 1) {
        ddang_slosh_theta = x[7];
        ddang_slosh_psi = x[8];
        Q_E1 = x[9];
        Q_E2 = x[10];
        Q_E3 = x[11];
        Q_E4 = x[12];
    }

    ddrhoC_1 = cross(ddang_1, rhoC_1) + cross(dang_1, cross(dang_1, rhoC_1));  // Eq.(5-12)

    delete [] ff;
    delete [] x;
}

void Forces::gamma_beta() {
    double ang_e1_theta;
    double ang_e2_psi;
    double ang_e3_theta;
    double ang_e4_psi;
    double ang_slosh_theta = grab_ang_slosh_theta();
    double ang_slosh_psi = grab_ang_slosh_psi();
    if (TWD_flag == 1) {
        ang_e1_theta = grab_ang_e1_theta();
        ang_e2_psi = grab_ang_e2_psi();
        ang_e3_theta = grab_ang_e3_theta();
        ang_e4_psi = grab_ang_e4_psi();
    }

    // 1st generalized coordinate q1
    // Velocity coefficient
    gamma_b1_q1(0) = 1.0;
    gamma_b1_q1(1) = 0.0;
    gamma_b1_q1(2) = 0.0;

    // Angular velocity coefficients
    beta_b1_q1(0) = 0.0;
    beta_b1_q1(1) = 0.0;
    beta_b1_q1(2) = 0.0;

    // 2nd generalized coordinate q2
    gamma_b1_q2(0) = 0.0;
    gamma_b1_q2(1) = 1.0;
    gamma_b1_q2(2) = 0.0;

    // Angular velocity coefficients
    beta_b1_q2(0) = 0.0;
    beta_b1_q2(1) = 0.0;
    beta_b1_q2(2) = 0.0;

    // 3rd generalized coordinate q3
    gamma_b1_q3(0) = 0.0;
    gamma_b1_q3(1) = 0.0;
    gamma_b1_q3(2) = 1.0;

    // Angular velocity coefficients
    beta_b1_q3(0) = 0.0;
    beta_b1_q3(1) = 0.0;
    beta_b1_q3(2) = 0.0;

    // 4th generalized coordinate q4
    gamma_b1_q4(0) = 0.0;
    gamma_b1_q4(1) = 0.0;
    gamma_b1_q4(2) = 0.0;

    // Angular velocity coefficients
    beta_b1_q4(0) = 1.0;
    beta_b1_q4(1) = 0.0;
    beta_b1_q4(2) = 0.0;

    // 5th generalized coordinate q5
    gamma_b1_q5(0) = 0.0;
    gamma_b1_q5(1) = 0.0;
    gamma_b1_q5(2) = 0.0;

    // Angular velocity coefficients
    beta_b1_q5(0) = 0.0;
    beta_b1_q5(1) = 1.0;
    beta_b1_q5(2) = 0.0;

    // 6th generalized coordinate q6
    gamma_b1_q6(0) = 0.0;
    gamma_b1_q6(1) = 0.0;
    gamma_b1_q6(2) = 0.0;

    // Angular velocity coefficients
    beta_b1_q6(0) = 0.0;
    beta_b1_q6(1) = 0.0;
    beta_b1_q6(2) = 1.0;

    // Slosh mass generalized coordinate q4
    beta_slosh_q4(0) = cos(ang_slosh_theta) * cos(ang_slosh_psi);
    beta_slosh_q4(1) = -cos(ang_slosh_theta) * sin(ang_slosh_psi);
    beta_slosh_q4(2) = sin(ang_slosh_theta);

    // Slosh mass generalized coordinate q5
    beta_slosh_q5(0) = sin(ang_slosh_psi);
    beta_slosh_q5(1) = cos(ang_slosh_psi);
    beta_slosh_q5(2) = 0.0;

    // Slosh mass generalized coordinate q6
    beta_slosh_q6(0) = -sin(ang_slosh_theta) * cos(ang_slosh_psi);
    beta_slosh_q6(1) = sin(ang_slosh_theta) * sin(ang_slosh_psi);
    beta_slosh_q6(2) = cos(ang_slosh_theta);

    // Slosh mass generalized coordinate q7
    beta_slosh_q7(0) = sin(ang_slosh_psi);
    beta_slosh_q7(1) = cos(ang_slosh_psi);
    beta_slosh_q7(2) = 0.0;

    // Slosh mass generalized coordinate q8
    beta_slosh_q8(0) = 0.0;
    beta_slosh_q8(1) = 0.0;
    beta_slosh_q8(2) = 1.0;

    // S2 Engine 1 mass generalized coordinate q4
    beta_S2_e1_q4(0) = cos(ang_e1_theta);
    beta_S2_e1_q4(1) = 0.0;
    beta_S2_e1_q4(2) = sin(ang_e1_theta);

    // S2 Engine 1 mass generalized coordinate q5
    beta_S2_e1_q5(0) = 0.0;
    beta_S2_e1_q5(1) = 1.0;
    beta_S2_e1_q5(2) = 0.0;

    // S2 Engine 1 mass generalized coordinate q6
    beta_S2_e1_q6(0) = -sin(ang_e1_theta);
    beta_S2_e1_q6(1) = 0.0;
    beta_S2_e1_q6(2) = cos(ang_e1_theta);

    // S2 Engine 1 mass generalized coordinate q7
    beta_S2_e1_q_theta(0) = 0.0;
    beta_S2_e1_q_theta(1) = 1.0;
    beta_S2_e1_q_theta(2) = 0.0;

    // S2 Engine 2 mass generalized coordinate q4
    beta_S2_e2_q4(0) = cos(ang_e2_psi);
    beta_S2_e2_q4(1) = -sin(ang_e2_psi);
    beta_S2_e2_q4(2) = 0.0;

    // S2 Engine 2 mass generalized coordinate q5
    beta_S2_e2_q5(0) = sin(ang_e2_psi);
    beta_S2_e2_q5(1) = cos(ang_e2_psi);
    beta_S2_e2_q5(2) = 0.0;

    // S2 Engine 2 mass generalized coordinate q6
    beta_S2_e2_q6(0) = 0.0;
    beta_S2_e2_q6(1) = 0.0;
    beta_S2_e2_q6(2) = 1.0;

    // S2 Engine 2 mass generalized coordinate q7
    beta_S2_e2_q_psi(0) = 0.0;
    beta_S2_e2_q_psi(1) = 0.0;
    beta_S2_e2_q_psi(2) = 1.0;

    // S2 Engine 3 mass generalized coordinate q4
    beta_S2_e3_q4(0) = cos(ang_e3_theta);
    beta_S2_e3_q4(1) = 0.0;
    beta_S2_e3_q4(2) = sin(ang_e3_theta);

    // S2 Engine 3 mass generalized coordinate q5
    beta_S2_e3_q5(0) = 0.0;
    beta_S2_e3_q5(1) = 1.0;
    beta_S2_e3_q5(2) = 0.0;

    // S2 Engine 3 mass generalized coordinate q6
    beta_S2_e3_q6(0) = -sin(ang_e3_theta);
    beta_S2_e3_q6(1) = 0.0;
    beta_S2_e3_q6(2) = cos(ang_e3_theta);

    // S2 Engine 3 mass generalized coordinate q7
    beta_S2_e3_q_theta(0) = 0.0;
    beta_S2_e3_q_theta(1) = 1.0;
    beta_S2_e3_q_theta(2) = 0.0;

    // S2 Engine 4 mass generalized coordinate q4
    beta_S2_e4_q4(0) = cos(ang_e4_psi);
    beta_S2_e4_q4(1) = -sin(ang_e4_psi);
    beta_S2_e4_q4(2) = 0.0;

    // S2 Engine 4 mass generalized coordinate q5
    beta_S2_e4_q5(0) = sin(ang_e4_psi);
    beta_S2_e4_q5(1) = cos(ang_e4_psi);
    beta_S2_e4_q5(2) = 0.0;

    // S2 Engine 4 mass generalized coordinate q6
    beta_S2_e4_q6(0) = 0.0;
    beta_S2_e4_q6(1) = 0.0;
    beta_S2_e4_q6(2) = 1.0;

    // S2 Engine 4 mass generalized coordinate q7
    beta_S2_e4_q_psi(0) = 0.0;
    beta_S2_e4_q_psi(1) = 0.0;
    beta_S2_e4_q_psi(2) = 1.0;
}

void Forces::AeroDynamics_Q() {
    double pdynmc = grab_pdynmc();
    double refa = grab_refa();
    double refd = grab_refd();
    double cy   = grab_cy();
    double cll  = grab_cll();
    double clm  = grab_clm();
    double cln  = grab_cln();
    double cx   = grab_cx();
    double cz   = grab_cz();
    arma::vec3 xcp  = grab_xcp();
    arma::mat33 TBI = grab_TBI();
    arma::vec3 rhoCP;
    rhoCP(0) = -(xcp(0) * refd) - (xp);
    rhoCP(1) = 0.0;
    rhoCP(2) = 0.0;


    // total non-gravitational forces
    FAPB = pdynmc * refa * arma::vec({cx, cy, cz});

    // aerodynamic moment
    FMAB = pdynmc * refa * refd * arma::vec({cll, clm, cln});

    Q_Aero(0) = dot(trans(TBI) * FAPB, gamma_b1_q1);
    Q_Aero(1) = dot(trans(TBI) * FAPB, gamma_b1_q2);
    Q_Aero(2) = dot(trans(TBI) * FAPB, gamma_b1_q3);
    Q_Aero(3) = dot(FMAB, beta_b1_q4) + dot(trans(TBI) * FAPB, -trans(TBI) * cross_matrix(rhoCP) * beta_b1_q4);
    Q_Aero(4) = dot(FMAB, beta_b1_q5) + dot(trans(TBI) * FAPB, -trans(TBI) * cross_matrix(rhoCP) * beta_b1_q5);
    Q_Aero(5) = dot(FMAB, beta_b1_q6) + dot(trans(TBI) * FAPB, -trans(TBI) * cross_matrix(rhoCP) * beta_b1_q6);
}

void Forces::Gravity_Q() {
    arma::vec3 GRAVG = grab_GRAVG();
    arma::vec3 NEXT_ACC = grab_NEXT_ACC();
    arma::mat33 TBI = grab_TBI();
    double vmass = grab_vmass();
    double thrust = grab_thrust();
    unsigned int liftoff = grab_liftoff();
    double e1_mass = grab_e1_mass();
    double e2_mass = grab_e2_mass();
    double e3_mass = grab_e3_mass();
    double e4_mass = grab_e4_mass();
    arma::vec3 Fg;
    Fg = vmass * GRAVG;

    if (liftoff == 1) {
        if (Slosh_flag == 1) {
            Fg = (vmass - slosh_mass) * GRAVG;
            Q_G_slosh(0) = dot(slosh_mass * (GRAVG), gamma_b1_q1);
            Q_G_slosh(1) = dot(slosh_mass * (GRAVG), gamma_b1_q2);
            Q_G_slosh(2) = dot(slosh_mass * (GRAVG), gamma_b1_q3);
            Q_G_slosh(3) = dot(slosh_mass * (GRAVG), -trans(TBI) * cross_matrix(r_slosh) * beta_b1_q4) + dot(slosh_mass * (GRAVG), -trans(TBSLOSH_I) * cross_matrix(rhoC_slosh) * beta_slosh_q4);
            Q_G_slosh(4) = dot(slosh_mass * (GRAVG), -trans(TBI) * cross_matrix(r_slosh) * beta_b1_q5) + dot(slosh_mass * (GRAVG), -trans(TBSLOSH_I) * cross_matrix(rhoC_slosh) * beta_slosh_q5);
            Q_G_slosh(5) = dot(slosh_mass * (GRAVG), -trans(TBI) * cross_matrix(r_slosh) * beta_b1_q6) + dot(slosh_mass * (GRAVG), -trans(TBSLOSH_I) * cross_matrix(rhoC_slosh) * beta_slosh_q6);
            Q_G_slosh(6) = dot(slosh_mass * (GRAVG), -trans(TBSLOSH_I) * cross_matrix(rhoC_slosh) * beta_slosh_q7);
            Q_G_slosh(7) = dot(slosh_mass * (GRAVG), -trans(TBSLOSH_I) * cross_matrix(rhoC_slosh) * beta_slosh_q8);
        }
        if (TWD_flag == 1) {
            Q_G_S2_E(0) = dot(e1_mass * GRAVG, gamma_b1_q1) + dot(e2_mass * GRAVG, gamma_b1_q1) + dot(e3_mass * GRAVG, gamma_b1_q1) + dot(e4_mass * GRAVG, gamma_b1_q1);
            Q_G_S2_E(1) = dot(e1_mass * GRAVG, gamma_b1_q2) + dot(e2_mass * GRAVG, gamma_b1_q2) + dot(e3_mass * GRAVG, gamma_b1_q2) + dot(e4_mass * GRAVG, gamma_b1_q2);
            Q_G_S2_E(2) = dot(e1_mass * GRAVG, gamma_b1_q3) + dot(e2_mass * GRAVG, gamma_b1_q3) + dot(e3_mass * GRAVG, gamma_b1_q3) + dot(e4_mass * GRAVG, gamma_b1_q3);
            Q_G_S2_E(3) = dot(e1_mass * GRAVG, -trans(TBI) * cross_matrix(e1_d) * beta_b1_q4) + dot(e1_mass * GRAVG, -trans(TE1_I) * cross_matrix(rhoC_e1) * beta_S2_e1_q4)
                            + dot(e2_mass * GRAVG, -trans(TBI) * cross_matrix(e2_d) * beta_b1_q4) + dot(e2_mass * GRAVG, -trans(TE2_I) * cross_matrix(rhoC_e2) * beta_S2_e2_q4)
                            + dot(e3_mass * GRAVG, -trans(TBI) * cross_matrix(e3_d) * beta_b1_q4) + dot(e3_mass * GRAVG, -trans(TE3_I) * cross_matrix(rhoC_e3) * beta_S2_e3_q4)
                            + dot(e4_mass * GRAVG, -trans(TBI) * cross_matrix(e4_d) * beta_b1_q4) + dot(e4_mass * GRAVG, -trans(TE4_I) * cross_matrix(rhoC_e4) * beta_S2_e4_q4);
            Q_G_S2_E(4) = dot(e1_mass * GRAVG, -trans(TBI) * cross_matrix(e1_d) * beta_b1_q5) + dot(e1_mass * GRAVG, -trans(TE1_I) * cross_matrix(rhoC_e1) * beta_S2_e1_q5)
                            + dot(e2_mass * GRAVG, -trans(TBI) * cross_matrix(e2_d) * beta_b1_q5) + dot(e2_mass * GRAVG, -trans(TE2_I) * cross_matrix(rhoC_e2) * beta_S2_e2_q5)
                            + dot(e3_mass * GRAVG, -trans(TBI) * cross_matrix(e3_d) * beta_b1_q5) + dot(e3_mass * GRAVG, -trans(TE3_I) * cross_matrix(rhoC_e3) * beta_S2_e3_q5)
                            + dot(e4_mass * GRAVG, -trans(TBI) * cross_matrix(e4_d) * beta_b1_q5) + dot(e4_mass * GRAVG, -trans(TE4_I) * cross_matrix(rhoC_e4) * beta_S2_e4_q5);
            Q_G_S2_E(5) = dot(e1_mass * GRAVG, -trans(TBI) * cross_matrix(e1_d) * beta_b1_q6) + dot(e1_mass * GRAVG, -trans(TE1_I) * cross_matrix(rhoC_e1) * beta_S2_e1_q6)
                            + dot(e2_mass * GRAVG, -trans(TBI) * cross_matrix(e2_d) * beta_b1_q6) + dot(e2_mass * GRAVG, -trans(TE2_I) * cross_matrix(rhoC_e2) * beta_S2_e2_q6)
                            + dot(e3_mass * GRAVG, -trans(TBI) * cross_matrix(e3_d) * beta_b1_q6) + dot(e3_mass * GRAVG, -trans(TE3_I) * cross_matrix(rhoC_e3) * beta_S2_e3_q6)
                            + dot(e4_mass * GRAVG, -trans(TBI) * cross_matrix(e4_d) * beta_b1_q6) + dot(e4_mass * GRAVG, -trans(TE4_I) * cross_matrix(rhoC_e4) * beta_S2_e4_q6);
            Q_G_S2_E(6) = dot(e1_mass * GRAVG, -trans(TE1_I) * cross_matrix(rhoC_e1) * beta_S2_e1_q_theta);
            Q_G_S2_E(7) = dot(e2_mass * GRAVG, -trans(TE2_I) * cross_matrix(rhoC_e2) * beta_S2_e2_q_psi);
            Q_G_S2_E(8) = dot(e3_mass * GRAVG, -trans(TE3_I) * cross_matrix(rhoC_e3) * beta_S2_e3_q_theta);
            Q_G_S2_E(9) = dot(e4_mass * GRAVG, -trans(TE4_I) * cross_matrix(rhoC_e4) * beta_S2_e4_q_psi);
            Fg = (vmass - e1_mass - e2_mass - e3_mass - e4_mass) * GRAVG;
        }
        if (Slosh_flag == 1 && TWD_flag == 1) {
            Fg = (vmass - e1_mass - e2_mass - e3_mass - e4_mass - slosh_mass) * GRAVG;
        }
        Q_G(0) = dot(Fg, gamma_b1_q1);
        Q_G(1) = dot(Fg, gamma_b1_q2);
        Q_G(2) = dot(Fg, gamma_b1_q3);
        Q_G(3) = dot(Fg, -trans(TBI) * cross_matrix(rhoC_1) * beta_b1_q4);
        Q_G(4) = dot(Fg, -trans(TBI) * cross_matrix(rhoC_1) * beta_b1_q5);
        Q_G(5) = dot(Fg, -trans(TBI) * cross_matrix(rhoC_1) * beta_b1_q6);
    } else {
        if (Slosh_flag == 1) {
            Q_G_slosh(0) = dot(slosh_mass * (NEXT_ACC), gamma_b1_q1);
            Q_G_slosh(1) = dot(slosh_mass * (NEXT_ACC), gamma_b1_q2);
            Q_G_slosh(2) = dot(slosh_mass * (NEXT_ACC), gamma_b1_q3);
            Q_G_slosh(3) = dot(slosh_mass * (NEXT_ACC), -trans(TBI) * cross_matrix(r_slosh) * beta_b1_q4) + dot(slosh_mass * (NEXT_ACC), -trans(TBSLOSH_I) * cross_matrix(rhoC_slosh) * beta_slosh_q4);
            Q_G_slosh(4) = dot(slosh_mass * (NEXT_ACC), -trans(TBI) * cross_matrix(r_slosh) * beta_b1_q5) + dot(slosh_mass * (NEXT_ACC), -trans(TBSLOSH_I) * cross_matrix(rhoC_slosh) * beta_slosh_q5);
            Q_G_slosh(5) = dot(slosh_mass * (NEXT_ACC), -trans(TBI) * cross_matrix(r_slosh) * beta_b1_q6) + dot(slosh_mass * (NEXT_ACC), -trans(TBSLOSH_I) * cross_matrix(rhoC_slosh) * beta_slosh_q6);
            Q_G_slosh(6) = dot(slosh_mass * (NEXT_ACC), -trans(TBSLOSH_I) * cross_matrix(rhoC_slosh) * beta_slosh_q7);
            Q_G_slosh(7) = dot(slosh_mass * (NEXT_ACC), -trans(TBSLOSH_I) * cross_matrix(rhoC_slosh) * beta_slosh_q8);
            Fg = (vmass - slosh_mass) * NEXT_ACC;
        }
        if (TWD_flag == 1) {
            Q_G_S2_E(0) = dot(e1_mass * NEXT_ACC, gamma_b1_q1) + dot(e2_mass * NEXT_ACC, gamma_b1_q1) + dot(e3_mass * NEXT_ACC, gamma_b1_q1) + dot(e4_mass * NEXT_ACC, gamma_b1_q1);
            Q_G_S2_E(1) = dot(e1_mass * NEXT_ACC, gamma_b1_q2) + dot(e2_mass * NEXT_ACC, gamma_b1_q2) + dot(e3_mass * NEXT_ACC, gamma_b1_q2) + dot(e4_mass * NEXT_ACC, gamma_b1_q2);
            Q_G_S2_E(2) = dot(e1_mass * NEXT_ACC, gamma_b1_q3) + dot(e2_mass * NEXT_ACC, gamma_b1_q3) + dot(e3_mass * NEXT_ACC, gamma_b1_q3) + dot(e4_mass * NEXT_ACC, gamma_b1_q3);
            Q_G_S2_E(3) = dot(e1_mass * NEXT_ACC, -trans(TBI) * cross_matrix(e1_d) * beta_b1_q4) + dot(e1_mass * NEXT_ACC, -trans(TE1_I) * cross_matrix(rhoC_e1) * beta_S2_e1_q4)
                            + dot(e2_mass * NEXT_ACC, -trans(TBI) * cross_matrix(e2_d) * beta_b1_q4) + dot(e2_mass * NEXT_ACC, -trans(TE2_I) * cross_matrix(rhoC_e2) * beta_S2_e2_q4)
                            + dot(e3_mass * NEXT_ACC, -trans(TBI) * cross_matrix(e3_d) * beta_b1_q4) + dot(e3_mass * NEXT_ACC, -trans(TE3_I) * cross_matrix(rhoC_e3) * beta_S2_e3_q4)
                            + dot(e4_mass * NEXT_ACC, -trans(TBI) * cross_matrix(e4_d) * beta_b1_q4) + dot(e4_mass * NEXT_ACC, -trans(TE4_I) * cross_matrix(rhoC_e4) * beta_S2_e4_q4);
            Q_G_S2_E(4) = dot(e1_mass * NEXT_ACC, -trans(TBI) * cross_matrix(e1_d) * beta_b1_q5) + dot(e1_mass * NEXT_ACC, -trans(TE1_I) * cross_matrix(rhoC_e1) * beta_S2_e1_q5)
                            + dot(e2_mass * NEXT_ACC, -trans(TBI) * cross_matrix(e2_d) * beta_b1_q5) + dot(e2_mass * NEXT_ACC, -trans(TE2_I) * cross_matrix(rhoC_e2) * beta_S2_e2_q5)
                            + dot(e3_mass * NEXT_ACC, -trans(TBI) * cross_matrix(e3_d) * beta_b1_q5) + dot(e3_mass * NEXT_ACC, -trans(TE3_I) * cross_matrix(rhoC_e3) * beta_S2_e3_q5)
                            + dot(e4_mass * NEXT_ACC, -trans(TBI) * cross_matrix(e4_d) * beta_b1_q5) + dot(e4_mass * NEXT_ACC, -trans(TE4_I) * cross_matrix(rhoC_e4) * beta_S2_e4_q5);
            Q_G_S2_E(5) = dot(e1_mass * NEXT_ACC, -trans(TBI) * cross_matrix(e1_d) * beta_b1_q6) + dot(e1_mass * NEXT_ACC, -trans(TE1_I) * cross_matrix(rhoC_e1) * beta_S2_e1_q6)
                            + dot(e2_mass * NEXT_ACC, -trans(TBI) * cross_matrix(e2_d) * beta_b1_q6) + dot(e2_mass * NEXT_ACC, -trans(TE2_I) * cross_matrix(rhoC_e2) * beta_S2_e2_q6)
                            + dot(e3_mass * NEXT_ACC, -trans(TBI) * cross_matrix(e3_d) * beta_b1_q6) + dot(e3_mass * NEXT_ACC, -trans(TE3_I) * cross_matrix(rhoC_e3) * beta_S2_e3_q6)
                            + dot(e4_mass * NEXT_ACC, -trans(TBI) * cross_matrix(e4_d) * beta_b1_q6) + dot(e4_mass * NEXT_ACC, -trans(TE4_I) * cross_matrix(rhoC_e4) * beta_S2_e4_q6);
            Q_G_S2_E(6) = dot(e1_mass * NEXT_ACC, -trans(TE1_I) * cross_matrix(rhoC_e1) * beta_S2_e1_q_theta);
            Q_G_S2_E(7) = dot(e2_mass * NEXT_ACC, -trans(TE2_I) * cross_matrix(rhoC_e2) * beta_S2_e2_q_psi);
            Q_G_S2_E(8) = dot(e3_mass * NEXT_ACC, -trans(TE3_I) * cross_matrix(rhoC_e3) * beta_S2_e3_q_theta);
            Q_G_S2_E(9) = dot(e4_mass * NEXT_ACC, -trans(TE4_I) * cross_matrix(rhoC_e4) * beta_S2_e4_q_psi);
            Fg = (vmass - e1_mass - e2_mass - e3_mass - e4_mass) * NEXT_ACC;
        }
        if (Slosh_flag == 1 && TWD_flag == 1) {
            Fg = (vmass - e1_mass - e2_mass - e3_mass - e4_mass - slosh_mass) * NEXT_ACC;
        }
        Q_G(0) = dot(Fg, gamma_b1_q1);
        Q_G(1) = dot(Fg, gamma_b1_q2);
        Q_G(2) = dot(Fg, gamma_b1_q3);
        Q_G(3) = dot(Fg, -trans(TBI) * cross_matrix(rhoC_1) * beta_b1_q4);
        Q_G(4) = dot(Fg, -trans(TBI) * cross_matrix(rhoC_1) * beta_b1_q5);
        Q_G(5) = dot(Fg, -trans(TBI) * cross_matrix(rhoC_1) * beta_b1_q6);
    }
}

void Forces::calculate_I1() {
    double vmass = grab_vmass();
    arma::mat33 IBBB = grab_IBBB();
    double e1_mass = grab_e1_mass();
    double e2_mass = grab_e2_mass();
    double e3_mass = grab_e3_mass();
    double e4_mass = grab_e4_mass();

    if (TWD_flag == 1) {
        I1(0, 0) = IBBB(0, 0) + (vmass - e1_mass - e2_mass - e3_mass - e4_mass) * (rhoC_1(1) * rhoC_1(1) + rhoC_1(2) * rhoC_1(2));
        I1(0, 1) = IBBB(0, 1) - (vmass - e1_mass - e2_mass - e3_mass - e4_mass) * rhoC_1(0) * rhoC_1(1);
        I1(0, 2) = IBBB(0, 2) - (vmass - e1_mass - e2_mass - e3_mass - e4_mass) * rhoC_1(0) * rhoC_1(2);
        I1(1, 0) = IBBB(1, 0) - (vmass - e1_mass - e2_mass - e3_mass - e4_mass) * rhoC_1(1) * rhoC_1(0);
        I1(1, 1) = IBBB(1, 1) + (vmass - e1_mass - e2_mass - e3_mass - e4_mass) * (rhoC_1(2) * rhoC_1(2) + rhoC_1(0) * rhoC_1(0));
        I1(1, 2) = IBBB(1, 2) - (vmass - e1_mass - e2_mass - e3_mass - e4_mass) * rhoC_1(1) * rhoC_1(2);
        I1(2, 0) = IBBB(2, 0) - (vmass - e1_mass - e2_mass - e3_mass - e4_mass) * rhoC_1(2) * rhoC_1(0);
        I1(2, 1) = IBBB(2, 1) - (vmass - e1_mass - e2_mass - e3_mass - e4_mass) * rhoC_1(2) * rhoC_1(1);
        I1(2, 2) = IBBB(2, 2) + (vmass - e1_mass - e2_mass - e3_mass - e4_mass) * (rhoC_1(0) * rhoC_1(0) + rhoC_1(1) * rhoC_1(1));
    } else {
        I1(0, 0) = IBBB(0, 0) + (vmass) * (rhoC_1(1) * rhoC_1(1) + rhoC_1(2) * rhoC_1(2));
        I1(0, 1) = IBBB(0, 1) - (vmass) * rhoC_1(0) * rhoC_1(1);
        I1(0, 2) = IBBB(0, 2) - (vmass) * rhoC_1(0) * rhoC_1(2);
        I1(1, 0) = IBBB(1, 0) - (vmass) * rhoC_1(1) * rhoC_1(0);
        I1(1, 1) = IBBB(1, 1) + (vmass) * (rhoC_1(2) * rhoC_1(2) + rhoC_1(0) * rhoC_1(0));
        I1(1, 2) = IBBB(1, 2) - (vmass) * rhoC_1(1) * rhoC_1(2);
        I1(2, 0) = IBBB(2, 0) - (vmass) * rhoC_1(2) * rhoC_1(0);
        I1(2, 1) = IBBB(2, 1) - (vmass) * rhoC_1(2) * rhoC_1(1);
        I1(2, 2) = IBBB(2, 2) + (vmass) * (rhoC_1(0) * rhoC_1(0) + rhoC_1(1) * rhoC_1(1));
    }
}

void Forces::calculate_I_E() {
    double e1_mass = grab_e1_mass();
    double e2_mass = grab_e2_mass();
    double e3_mass = grab_e3_mass();
    double e4_mass = grab_e4_mass();

    arma::mat33 I_S2_E1 = grab_I_S2_E1();
    arma::mat33 I_S2_E2 = grab_I_S2_E2();
    arma::mat33 I_S2_E3 = grab_I_S2_E3();
    arma::mat33 I_S2_E4 = grab_I_S2_E4();

    I_E1(0, 0) = I_S2_E1(0, 0) + e1_mass * (rhoC_e1(1) * rhoC_e1(1) + rhoC_e1(2) * rhoC_e1(2));
    I_E1(0, 1) = I_S2_E1(0, 1) - e1_mass * rhoC_e1(0) * rhoC_e1(1);
    I_E1(0, 2) = I_S2_E1(0, 2) - e1_mass * rhoC_e1(0) * rhoC_e1(2);
    I_E1(1, 0) = I_S2_E1(1, 0) - e1_mass * rhoC_e1(1) * rhoC_e1(0);
    I_E1(1, 1) = I_S2_E1(1, 1) + e1_mass * (rhoC_e1(2) * rhoC_e1(2) + rhoC_e1(0) * rhoC_e1(0));
    I_E1(1, 2) = I_S2_E1(1, 2) - e1_mass * rhoC_e1(1) * rhoC_e1(2);
    I_E1(2, 0) = I_S2_E1(2, 0) - e1_mass * rhoC_e1(2) * rhoC_e1(0);
    I_E1(2, 1) = I_S2_E1(2, 1) - e1_mass * rhoC_e1(2) * rhoC_e1(1);
    I_E1(2, 2) = I_S2_E1(2, 2) + e1_mass * (rhoC_e1(0) * rhoC_e1(0) + rhoC_e1(1) * rhoC_e1(1));

    I_E2(0, 0) = I_S2_E2(0, 0) + e2_mass * (rhoC_e2(1) * rhoC_e2(1) + rhoC_e2(2) * rhoC_e2(2));
    I_E2(0, 1) = I_S2_E2(0, 1) - e2_mass * rhoC_e2(0) * rhoC_e2(1);
    I_E2(0, 2) = I_S2_E2(0, 2) - e2_mass * rhoC_e2(0) * rhoC_e2(2);
    I_E2(1, 0) = I_S2_E2(1, 0) - e2_mass * rhoC_e2(1) * rhoC_e2(0);
    I_E2(1, 1) = I_S2_E2(1, 1) + e2_mass * (rhoC_e2(2) * rhoC_e2(2) + rhoC_e2(0) * rhoC_e2(0));
    I_E2(1, 2) = I_S2_E2(1, 2) - e2_mass * rhoC_e2(1) * rhoC_e2(2);
    I_E2(2, 0) = I_S2_E2(2, 0) - e2_mass * rhoC_e2(2) * rhoC_e2(0);
    I_E2(2, 1) = I_S2_E2(2, 1) - e2_mass * rhoC_e2(2) * rhoC_e2(1);
    I_E2(2, 2) = I_S2_E2(2, 2) + e2_mass * (rhoC_e2(0) * rhoC_e2(0) + rhoC_e2(1) * rhoC_e2(1));

    I_E3(0, 0) = I_S2_E3(0, 0) + e3_mass * (rhoC_e3(1) * rhoC_e3(1) + rhoC_e3(2) * rhoC_e3(2));
    I_E3(0, 1) = I_S2_E3(0, 1) - e3_mass * rhoC_e3(0) * rhoC_e3(1);
    I_E3(0, 2) = I_S2_E3(0, 2) - e3_mass * rhoC_e3(0) * rhoC_e3(2);
    I_E3(1, 0) = I_S2_E3(1, 0) - e3_mass * rhoC_e3(1) * rhoC_e3(0);
    I_E3(1, 1) = I_S2_E3(1, 1) + e3_mass * (rhoC_e3(2) * rhoC_e3(2) + rhoC_e3(0) * rhoC_e3(0));
    I_E3(1, 2) = I_S2_E3(1, 2) - e3_mass * rhoC_e3(1) * rhoC_e3(2);
    I_E3(2, 0) = I_S2_E3(2, 0) - e3_mass * rhoC_e3(2) * rhoC_e3(0);
    I_E3(2, 1) = I_S2_E3(2, 1) - e3_mass * rhoC_e3(2) * rhoC_e3(1);
    I_E3(2, 2) = I_S2_E3(2, 2) + e3_mass * (rhoC_e3(0) * rhoC_e3(0) + rhoC_e3(1) * rhoC_e3(1));

    I_E4(0, 0) = I_S2_E4(0, 0) + e4_mass * (rhoC_e4(1) * rhoC_e4(1) + rhoC_e4(2) * rhoC_e4(2));
    I_E4(0, 1) = I_S2_E4(0, 1) - e4_mass * rhoC_e4(0) * rhoC_e4(1);
    I_E4(0, 2) = I_S2_E4(0, 2) - e4_mass * rhoC_e4(0) * rhoC_e4(2);
    I_E4(1, 0) = I_S2_E4(1, 0) - e4_mass * rhoC_e4(1) * rhoC_e4(0);
    I_E4(1, 1) = I_S2_E4(1, 1) + e4_mass * (rhoC_e4(2) * rhoC_e4(2) + rhoC_e4(0) * rhoC_e4(0));
    I_E4(1, 2) = I_S2_E4(1, 2) - e4_mass * rhoC_e4(1) * rhoC_e4(2);
    I_E4(2, 0) = I_S2_E4(2, 0) - e4_mass * rhoC_e4(2) * rhoC_e4(0);
    I_E4(2, 1) = I_S2_E4(2, 1) - e4_mass * rhoC_e4(2) * rhoC_e4(1);
    I_E4(2, 2) = I_S2_E4(2, 2) + e4_mass * (rhoC_e4(0) * rhoC_e4(0) + rhoC_e4(1) * rhoC_e4(1));
}

arma::mat33 Forces::cross_matrix(arma::vec3 in) {
    arma::mat33 c_matrix;

    c_matrix(0, 0) = 0.0;
    c_matrix(0, 1) = -in(2);
    c_matrix(0, 2) = in(1);
    c_matrix(1, 0) = in(2);
    c_matrix(1, 1) = 0.0;
    c_matrix(1, 2) = -in(0);
    c_matrix(2, 0) = -in(1);
    c_matrix(2, 1) = in(0);
    c_matrix(2, 2) = 0.0;

    return c_matrix;
}

void Forces::funcv(int n, double *x, double *ff) {
    arma::mat33 TB1_I = grab_TBI();
    double m1 = grab_vmass();
    arma::vec6 Q_TVC = grab_Q_TVC();

    for (int i = 0; i < 3; i++) {
        ddrP_1(i) = x[i + 1];
        ddang_1(i) = x[i + 4];
    }

    if (Slosh_flag == 1 && TWD_flag == 0) {
        ddang_slosh_theta = x[7];
        ddang_slosh_psi = x[8];
        // Q_slosh_7 = x[7];  // to constrain slosh mass not to move
        // Q_slosh_8 = x[8];  // to constrain slsoh mass not to move
        slosh();
        m1 = m1 - slosh_mass;
    } else if (TWD_flag == 1 && Slosh_flag == 0) {
        double e1_mass = grab_e1_mass();
        double e2_mass = grab_e2_mass();
        double e3_mass = grab_e3_mass();
        double e4_mass = grab_e4_mass();

        Q_E1 = x[7];
        Q_E2 = x[8];
        Q_E3 = x[9];
        Q_E4 = x[10];

        S2_TWD();
        m1 = m1 - e1_mass - e2_mass - e3_mass - e4_mass;
    } else if (Slosh_flag == 1 && TWD_flag == 1) {
        double e1_mass = grab_e1_mass();
        double e2_mass = grab_e2_mass();
        double e3_mass = grab_e3_mass();
        double e4_mass = grab_e4_mass();
        ddang_slosh_theta = x[7];
        ddang_slosh_psi = x[8];
        Q_E1 = x[9];
        Q_E2 = x[10];
        Q_E3 = x[11];
        Q_E4 = x[12];
        slosh();
        S2_TWD();
        m1 = m1 - e1_mass - e2_mass - e3_mass - e4_mass - slosh_mass;
    }

    ddrhoC_1 = cross(ddang_1, rhoC_1) + cross(dang_1, cross(dang_1, rhoC_1));  // Eq.(5-12)
    //  Eq.(5-19)
    p_b1_ga = m1 * (ddrP_1 + trans(TB1_I) * ddrhoC_1);
    p_b1_be = I1 * ddang_1 + cross_matrix(dang_1) * I1 * dang_1 + m1 * cross_matrix(rhoC_1) * TB1_I * ddrP_1;
    f(0) = dot(p_b1_ga, gamma_b1_q1) - (Q_G(0) + Q_TVC(0) + Q_Aero(0));
    f(1) = dot(p_b1_ga, gamma_b1_q2) - (Q_G(1) + Q_TVC(1) + Q_Aero(1));
    f(2) = dot(p_b1_ga, gamma_b1_q3) - (Q_G(2) + Q_TVC(2) + Q_Aero(2));
    f(3) = dot(p_b1_be, beta_b1_q4) - (Q_G(3) + Q_TVC(3) + Q_Aero(3));
    f(4) = dot(p_b1_be, beta_b1_q5) - (Q_G(4) + Q_TVC(4) + Q_Aero(4));
    f(5) = dot(p_b1_be, beta_b1_q6) - (Q_G(5) + Q_TVC(5) + Q_Aero(5));

    if (Slosh_flag == 1 && TWD_flag == 0) {
        f(0) = dot(p_b1_ga, gamma_b1_q1) + f_slosh(0) - (Q_G(0) + Q_TVC(0) + Q_Aero(0) + Q_G_slosh(0));
        f(1) = dot(p_b1_ga, gamma_b1_q2) + f_slosh(1) - (Q_G(1) + Q_TVC(1) + Q_Aero(1) + Q_G_slosh(1));
        f(2) = dot(p_b1_ga, gamma_b1_q3) + f_slosh(2) - (Q_G(2) + Q_TVC(2) + Q_Aero(2) + Q_G_slosh(2));
        f(3) = dot(p_b1_be, beta_b1_q4) + f_slosh(3) - (Q_G(3) + Q_TVC(3) + Q_Aero(3) + Q_G_slosh(3));
        f(4) = dot(p_b1_be, beta_b1_q5) + f_slosh(4) - (Q_G(4) + Q_TVC(4) + Q_Aero(4) + Q_G_slosh(4));
        f(5) = dot(p_b1_be, beta_b1_q6) + f_slosh(5) - (Q_G(5) + Q_TVC(5) + Q_Aero(5) + Q_G_slosh(5));
        f(6) = f_slosh(6) - Q_G_slosh(6);
        f(7) = f_slosh(7) - Q_G_slosh(7);
    } else if (TWD_flag == 1 && Slosh_flag == 0) {
        f(0) = dot(p_b1_ga, gamma_b1_q1) + f_S2_TWD(0) - (Q_G(0) + Q_TVC(0) + Q_Aero(0) + Q_G_S2_E(0));
        f(1) = dot(p_b1_ga, gamma_b1_q2) + f_S2_TWD(1) - (Q_G(1) + Q_TVC(1) + Q_Aero(1) + Q_G_S2_E(1));
        f(2) = dot(p_b1_ga, gamma_b1_q3) + f_S2_TWD(2) - (Q_G(2) + Q_TVC(2) + Q_Aero(2) + Q_G_S2_E(2));
        f(3) = dot(p_b1_be, beta_b1_q4) + f_S2_TWD(3) - (Q_G(3) + Q_TVC(3) + Q_Aero(3) + Q_G_S2_E(3));
        f(4) = dot(p_b1_be, beta_b1_q5) + f_S2_TWD(4) - (Q_G(4) + Q_TVC(4) + Q_Aero(4) + Q_G_S2_E(4));
        f(5) = dot(p_b1_be, beta_b1_q6) + f_S2_TWD(5) - (Q_G(5) + Q_TVC(5) + Q_Aero(5) + Q_G_S2_E(5));
        f(6) = f_S2_TWD(6) - Q_G_S2_E(6);
        f(7) = f_S2_TWD(7) - Q_G_S2_E(7);
        f(8) = f_S2_TWD(8) - Q_G_S2_E(8);
        f(9) = f_S2_TWD(9) - Q_G_S2_E(9);
    } else if (Slosh_flag == 1 && TWD_flag == 1) {
        f(0) = dot(p_b1_ga, gamma_b1_q1) + f_slosh(0) + f_S2_TWD(0) - (Q_G(0) + Q_TVC(0) + Q_Aero(0) + Q_G_slosh(0) + Q_G_S2_E(0));
        f(1) = dot(p_b1_ga, gamma_b1_q2) + f_slosh(1) + f_S2_TWD(1) - (Q_G(1) + Q_TVC(1) + Q_Aero(1) + Q_G_slosh(1) + Q_G_S2_E(1));
        f(2) = dot(p_b1_ga, gamma_b1_q3) + f_slosh(2) + f_S2_TWD(2) - (Q_G(2) + Q_TVC(2) + Q_Aero(2) + Q_G_slosh(2) + Q_G_S2_E(2));
        f(3) = dot(p_b1_be, beta_b1_q4) + f_slosh(3) + f_S2_TWD(3) - (Q_G(3) + Q_TVC(3) + Q_Aero(3) + Q_G_slosh(3) + Q_G_S2_E(3));
        f(4) = dot(p_b1_be, beta_b1_q5) + f_slosh(4) + f_S2_TWD(4) - (Q_G(4) + Q_TVC(4) + Q_Aero(4) + Q_G_slosh(4) + Q_G_S2_E(4));
        f(5) = dot(p_b1_be, beta_b1_q6) + f_slosh(5) + f_S2_TWD(5) - (Q_G(5) + Q_TVC(5) + Q_Aero(5) + Q_G_slosh(5) + Q_G_S2_E(5));
        f(6) = f_slosh(6) - Q_G_slosh(6);
        f(7) = f_slosh(7) - Q_G_slosh(7);
        f(8) = f_S2_TWD(6) - Q_G_S2_E(6);
        f(9) = f_S2_TWD(7) - Q_G_S2_E(7);
        f(10) = f_S2_TWD(8) - Q_G_S2_E(8);
        f(11) = f_S2_TWD(9) - Q_G_S2_E(9);
    }

    for (int i = 0; i < n; i++) {
        ff[i + 1] = f(i);
    }
}

void Forces::slosh() {
    arma::vec3 p_slosh_ga, p_slosh_be, domega_slosh1_B, domega_slosh2_slosh1;
    arma::mat33 TBI = grab_TBI();
    // arma::vec3 NEXT_ACC = grab_NEXT_ACC();

    // ddang_slosh_theta = 0.0;
    // ddang_slosh_psi = 0.0;

    domega_slosh1_B(0) = 0.0;
    domega_slosh1_B(1) = ddang_slosh_theta;
    domega_slosh1_B(2) = 0.0;

    domega_slosh2_slosh1(0) = 0.0;
    domega_slosh2_slosh1(1) = 0.0;
    domega_slosh2_slosh1(2) = ddang_slosh_psi;

    ddrhoC_slosh = cross(ddang_1, r_slosh) + cross(dang_1, cross(dang_1, r_slosh));  // Eq.(5-64)

    /***********************************************/

    ddang_slosh = TSLOSHB2_SLOSHB1 * (TSLOSHB1_B * ddang_1 + domega_slosh1_B + cross_matrix(dang_slosh1) * omega_slosh1_B)
                     + domega_slosh2_slosh1 + cross_matrix(dang_slosh2) * omega_slosh2_slosh1;

    /************************************************/

    ddrP_slosh = ddrP_1 + trans(TBI) * ddrhoC_slosh
                 + trans(TBSLOSH_I) * (cross(ddang_slosh, rhoC_slosh) + cross(dang_slosh2, cross(dang_slosh2, rhoC_slosh)));  // Eq.(5-12)

    p_slosh_ga = slosh_mass * (ddrP_slosh);  // inertail
    ddrP_sloshB = TBSLOSH_I * ddrP_slosh;

    C_c = 2. * slosh_mass * Wn;  // Slosh mass critical damping
    C_1 = damping_ratio * C_c;  // Slosh mass damping

    arma::vec3 QD7 = -C_1 * omega_slosh1_B;
    arma::vec3 QD8 = -C_1 * omega_slosh2_slosh1;

    f_slosh(0) = dot(p_slosh_ga, gamma_b1_q1);
    f_slosh(1) = dot(p_slosh_ga, gamma_b1_q2);
    f_slosh(2) = dot(p_slosh_ga, gamma_b1_q3);
    f_slosh(3) = dot(p_slosh_ga, -trans(TBI) * cross_matrix(r_slosh) * beta_b1_q4) + dot(p_slosh_ga, -trans(TBSLOSH_I) * cross_matrix(rhoC_slosh) * beta_slosh_q4);
    f_slosh(4) = dot(p_slosh_ga, -trans(TBI) * cross_matrix(r_slosh) * beta_b1_q5) + dot(p_slosh_ga, -trans(TBSLOSH_I) * cross_matrix(rhoC_slosh) * beta_slosh_q5);
    f_slosh(5) = dot(p_slosh_ga, -trans(TBI) * cross_matrix(r_slosh) * beta_b1_q6) + dot(p_slosh_ga, -trans(TBSLOSH_I) * cross_matrix(rhoC_slosh) * beta_slosh_q6);
    f_slosh(6) = dot(p_slosh_ga, -trans(TBSLOSH_I) * cross_matrix(rhoC_slosh) * beta_slosh_q7) - dot(QD7, beta_slosh_q7);
    f_slosh(7) = dot(p_slosh_ga, -trans(TBSLOSH_I) * cross_matrix(rhoC_slosh) * beta_slosh_q8) - dot(QD8, beta_slosh_q8);
}

void Forces::S2_TWD() {
    arma::vec3 p_e1_ga, p_e2_ga, p_e3_ga, p_e4_ga, p_e1_be, p_e2_be, p_e3_be, p_e4_be
                , domega_e1_B, domega_e2_B, domega_e3_B, domega_e4_B;
    arma::mat33 TBI = grab_TBI();

    double e1_mass = grab_e1_mass();
    double e2_mass = grab_e2_mass();
    double e3_mass = grab_e3_mass();
    double e4_mass = grab_e4_mass();

    /*******************************************************************************************************/
    domega_e1_B(0) = 0.0;
    domega_e1_B(1) = ddang_e1_theta;
    domega_e1_B(2) = 0.0;

    domega_e2_B(0) = 0.0;
    domega_e2_B(1) = 0.0;
    domega_e2_B(2) = ddang_e2_psi;

    domega_e3_B(0) = 0.0;
    domega_e3_B(1) = ddang_e3_theta;
    domega_e3_B(2) = 0.0;

    domega_e4_B(0) = 0.0;
    domega_e4_B(1) = 0.0;
    domega_e4_B(2) = ddang_e4_psi;
    /*******************************************************************************************************/
    ddrP_e1 = ddrP_1 + trans(TBI) * (cross(ddang_1, e1_d) + cross(dang_1, cross(dang_1, e1_d)));
    ddrP_e2 = ddrP_1 + trans(TBI) * (cross(ddang_1, e2_d) + cross(dang_1, cross(dang_1, e2_d)));
    ddrP_e3 = ddrP_1 + trans(TBI) * (cross(ddang_1, e3_d) + cross(dang_1, cross(dang_1, e3_d)));
    ddrP_e4 = ddrP_1 + trans(TBI) * (cross(ddang_1, e4_d) + cross(dang_1, cross(dang_1, e4_d)));
    /*******************************************************************************************************/
    domega_e1 = (TE1_B * ddang_1 - cross_matrix(dang_e1) * omega_e1_B + domega_e1_B);
    domega_e2 = (TE2_B * ddang_1 - cross_matrix(dang_e2) * omega_e2_B + domega_e2_B);
    domega_e3 = (TE3_B * ddang_1 - cross_matrix(dang_e3) * omega_e3_B + domega_e3_B);
    domega_e4 = (TE4_B * ddang_1 - cross_matrix(dang_e4) * omega_e4_B + domega_e4_B);
    /*******************************************************************************************************/
    ddrhoC_e1 = cross(domega_e1, rhoC_e1) + cross(dang_e1, cross(dang_e1, rhoC_e1));
    ddrhoC_e2 = cross(domega_e2, rhoC_e2) + cross(dang_e2, cross(dang_e2, rhoC_e2));
    ddrhoC_e3 = cross(domega_e3, rhoC_e3) + cross(dang_e3, cross(dang_e3, rhoC_e3));
    ddrhoC_e4 = cross(domega_e4, rhoC_e4) + cross(dang_e4, cross(dang_e4, rhoC_e4));
    /*******************************************************************************************************/
    p_e1_ga = e1_mass * (ddrP_e1 + trans(TE1_I) * ddrhoC_e1);
    p_e2_ga = e2_mass * (ddrP_e2 + trans(TE2_I) * ddrhoC_e2);
    p_e3_ga = e3_mass * (ddrP_e3 + trans(TE3_I) * ddrhoC_e3);
    p_e4_ga = e4_mass * (ddrP_e4 + trans(TE4_I) * ddrhoC_e4);
    /*******************************************************************************************************/
    p_e1_be = I_E1 * domega_e1 + cross_matrix(dang_e1) * I_E1 * dang_e1 +
                e1_mass * cross_matrix(rhoC_e1) * TE1_I * ddrP_e1;
    p_e2_be = I_E2 * domega_e2 + cross_matrix(dang_e2) * I_E2 * dang_e2 +
                e2_mass * cross_matrix(rhoC_e2) * TE2_I * ddrP_e2;
    p_e3_be = I_E3 * domega_e3 + cross_matrix(dang_e3) * I_E3 * dang_e3 +
                e3_mass * cross_matrix(rhoC_e3) * TE3_I * ddrP_e3;
    p_e4_be = I_E4 * domega_e4 + cross_matrix(dang_e4) * I_E4 * dang_e4 +
                e4_mass * cross_matrix(rhoC_e4) * TE4_I * ddrP_e4;
    /*******************************************************************************************************/
    f_S2_TWD(0) = dot(p_e1_ga, gamma_b1_q1) + dot(p_e2_ga, gamma_b1_q1) + dot(p_e3_ga, gamma_b1_q1) + dot(p_e4_ga, gamma_b1_q1);
    f_S2_TWD(1) = dot(p_e1_ga, gamma_b1_q2) + dot(p_e2_ga, gamma_b1_q2) + dot(p_e3_ga, gamma_b1_q2) + dot(p_e4_ga, gamma_b1_q2);
    f_S2_TWD(2) = dot(p_e1_ga, gamma_b1_q3) + dot(p_e2_ga, gamma_b1_q3) + dot(p_e3_ga, gamma_b1_q3) + dot(p_e4_ga, gamma_b1_q3);
    f_S2_TWD(3) = dot(p_e1_ga, -trans(TBI) * cross_matrix(e1_d) * beta_b1_q4) + dot(p_e1_be, beta_S2_e1_q4) + dot(p_e2_ga, -trans(TBI) * cross_matrix(e2_d) * beta_b1_q4) + dot(p_e2_be, beta_S2_e2_q4)
                    + dot(p_e3_ga, -trans(TBI) * cross_matrix(e3_d) * beta_b1_q4) + dot(p_e3_be, beta_S2_e3_q4) + dot(p_e4_ga, -trans(TBI) * cross_matrix(e4_d) * beta_b1_q4) + dot(p_e4_be, beta_S2_e4_q4);
    f_S2_TWD(4) = dot(p_e1_ga, -trans(TBI) * cross_matrix(e1_d) * beta_b1_q5) + dot(p_e1_be, beta_S2_e1_q5) + dot(p_e2_ga, -trans(TBI) * cross_matrix(e2_d) * beta_b1_q5) + dot(p_e2_be, beta_S2_e2_q5)
                    + dot(p_e3_ga, -trans(TBI) * cross_matrix(e3_d) * beta_b1_q5) + dot(p_e3_be, beta_S2_e3_q5) + dot(p_e4_ga, -trans(TBI) * cross_matrix(e4_d) * beta_b1_q5) + dot(p_e4_be, beta_S2_e4_q5);
    f_S2_TWD(5) = dot(p_e1_ga, -trans(TBI) * cross_matrix(e1_d) * beta_b1_q6) + dot(p_e1_be, beta_S2_e1_q6) + dot(p_e2_ga, -trans(TBI) * cross_matrix(e2_d) * beta_b1_q6) + dot(p_e2_be, beta_S2_e2_q6)
                    + dot(p_e3_ga, -trans(TBI) * cross_matrix(e3_d) * beta_b1_q6) + dot(p_e3_be, beta_S2_e3_q6) + dot(p_e4_ga, -trans(TBI) * cross_matrix(e4_d) * beta_b1_q6) + dot(p_e4_be, beta_S2_e4_q6);
    f_S2_TWD(6) = dot(p_e1_be, beta_S2_e1_q_theta) - Q_E1;
    f_S2_TWD(7) = dot(p_e2_be, beta_S2_e2_q_psi) - Q_E2;
    f_S2_TWD(8) = dot(p_e3_be, beta_S2_e3_q_theta) - Q_E3;
    f_S2_TWD(9) = dot(p_e4_be, beta_S2_e4_q_psi) - Q_E4;
}

void Forces::broydn(double x[], int n, int *check) {
    int i, j, k, restrt, sing, skip;
    double den, fff, fold, stpmax, sum, temp, test, *c, *d, *fvcold;
    double *g, *p, **qt, **r, *s, *t, *w, *xold;
    c = dvector(1, n);
    d = dvector(1, n);
    fvcold = dvector(1, n);
    g = dvector(1, n);
    p = dvector(1, n);
    qt = dmatrix(1, n, 1, n);
    r = dmatrix(1, n, 1, n);
    s = dvector(1, n);
    t = dvector(1, n);
    w = dvector(1, n);
    xold = dvector(1, n);
    fvec = dvector(1, n);
    nn = n;
    fff = f_min(x);
    test = 0.0;
    for (i = 1; i <= n; i++)
        if (fabs(fvec[i]) > test)test = fabs(fvec[i]);
    if (test < 0.01 * TOLF) {
        *check = 0;
        FREERETURN
    }
    for (sum = 0.0, i = 1; i <= n; i++) sum += DSQR(x[i]);
    stpmax = STPMX * DMAX(sqrt(sum), (double)n);
    restrt = 1;
    for (its = 1; its <= MAXITS; its++) {
        if (restrt) {
            fdjac(n, x, fvec, r);
            qrdcmp(r, n, c, d, &sing);
            if (sing) nrerror("singular Jacobian in broydn : qrdcmp");
            for (i = 1; i <= n; i++) {
                for (j = 1; j <= n; j++) qt[i][j] = 0.0;
                qt[i][i] = 1.0;
            }
            for (k = 1; k < n; k++) {
                if (c[k]) {
                    for (j = 1; j <= n; j++) {
                        sum = 0.0;
                        for (i = k; i <= n; i++)
                            sum += r[i][k] * qt[i][j];
                        sum /= c[k];
                        for (i = k; i <= n; i++)
                            qt[i][j] -= sum * r[i][k];
                    }
                }
            }
            for (i = 1; i <= n; i++) {
                r[i][i] = d[i];
                for (j = 1; j < i; j++) r[i][j] = 0.0;
            }
        } else {
            for (i = 1; i <= n; i++) s[i] = x[i] - xold[i];
            for (i = 1; i <= n; i++) {
                for (sum = 0.0, j = i; j <= n; j++) sum += r[i][j] * s[j];
                t[i] = sum;
            }
            skip = 1;
            for (i = 1; i <= n; i++) {
                for (sum = 0.0, j = 1; j <= n; j++) sum += qt[j][i] * t[j];
                w[i] = fvec[i] - fvcold[i] - sum;
                if (fabs(w[i]) >= EPS * (fabs(fvec[i]) + fabs(fvcold[i]))) {
                    skip = 0;
                } else {
                    w[i] = 0.0;
                }
            }
            if (!skip) {
                for (i = 1; i <= n; i++) {
                    for (sum = 0.0, j = 1; j <= n; j++) sum += qt[i][j] * w[j];
                    t[i] = sum;
                }
                for (den = 0.0, i = 1; i <= n; i++) den += DSQR(s[i]);
                for (i = 1; i <= n; i++) s[i] /= den;
                qrupdt(r, qt, n, t, s);
                for (i = 1; i <= n; i++) {
                    if (r[i][i] == 0.0) nrerror("r singular in broydn : qrupdt");
                    d[i] = r[i][i];
                }
            }
        }
        for (i = 1; i <= n; i++) {
            for (sum = 0.0, j = 1; j <= n; j++) sum += qt[i][j] * fvec[j];
            p[i] = -sum;
        }
        for (i = n; i >= 1; i--) {
            for (sum = 0.0, j = 1; j <= i; j++) sum -= r[j][i] * p[j];
            g[i] = sum;
        }
        for (i = 1; i <= n; i++) {
            xold[i] = x[i];
            fvcold[i] = fvec[i];
        }
        fold = fff;
        rsolv(r, n, d, p);
        lnsrch(n, xold, fold, g, p, x, &fff, stpmax, check);
        test = 0.0;
        for (i = 1; i <= n; i++)
            if (fabs(fvec[i]) > test) test = fabs(fvec[i]);
        if (test < TOLF) {
            *check = 0;
            FREERETURN
        }
        if (*check) {
            if (restrt) {
                FREERETURN
            } else {
                    test = 0.0;
                    den = DMAX(fff, 0.5 * n);
                    for (i = 1; i <= n; i++) {
                        temp = fabs(g[i]) * DMAX(fabs(x[i]), 1.0) / den;
                        if (temp > test) {
                            test = temp;
                        }
                    }
                    if (test < TOLMIN) {
                        FREERETURN
                    } else {
                        restrt = 1;
                    }
                }
        } else {
            restrt = 0;
            test = 0.0;
            for (i = 1; i <= n; i++) {
                temp = (fabs(x[i] - xold[i])) / DMAX(fabs(x[i]), 1.0);
                if (temp > test) test = temp;
            }
            if (test < EPS) FREERETURN
            }
    }
    nrerror("MAXITS exceeded in broydn");
    FREERETURN
}

void Forces::rsolv(double **a, int n, double d[], double b[]) {
    int i, j;
    double sum;

    b[n] /= d[n];
    for (i = n - 1; i >= 1; i--) {
        for (sum = 0.0, j = i + 1; j <= n; j++) sum += a[i][j] * b[j];
        b[i] = (b[i] - sum) / d[i];
    }
}

void Forces::fdjac(int n, double x[], double fvec[], double **df) {
    int i, j;
    double h, temp, *ff;

    ff = dvector(1, n);
    for (j = 1; j <= n; j++) {
        temp = x[j];
        h = EPS * fabs(temp);
        if (h == 0.0) h = EPS;
        x[j] = temp + h;
        h = x[j] - temp;
        funcv(n, x, ff);
        x[j] = temp;
        for (i = 1; i <= n; i++) {
            df[i][j] = (ff[i] - fvec[i]) / h;
        }
    }
    free_dvector(ff, 1, n);
}

double Forces::f_min(double x[]) {
    int i;
    double sum;

    funcv(nn, x, fvec);
    for (sum = 0.0, i = 1; i <= nn; i++) sum += DSQR(fvec[i]);
    return 0.5 * sum;
}

void Forces::lnsrch(int n, double xold[], double fold, double g[], double p[], double x[],
                    double *f, double stpmax, int *check) {
    int i;
    double a, alam, alam2, alamin, b, disc, f2, rhs1, rhs2, slope, sum, temp,
           test, tmplam;

    *check = 0;
    for (sum = 0.0, i = 1; i <= n; i++) sum += p[i] * p[i];
    sum = sqrt(sum);
    if (sum > stpmax) {
        for (i = 1; i <= n; i++) p[i] *= stpmax / sum;
    }
    for (slope = 0.0, i = 1; i <= n; i++)
        slope += g[i] * p[i];
    if (slope >= 0.0) {
        nrerror("Roundoff problem in lnsrch.");
    }
    test = 0.0;
    for (i = 1; i <= n; i++) {
        temp = fabs(p[i]) / DMAX(fabs(xold[i]), 1.0);
        if (temp > test) {
            test = temp;
        }
    }
    alamin = TOLX / test;
    alam = 1.0;
    for (;;) {
        for (i = 1; i <= n; i++) x[i] = xold[i] + alam * p[i];
        *f = f_min(x);
        if (alam < alamin) {
            for (i = 1; i <= n; i++) x[i] = xold[i];
            *check = 1;
            return;
        } else if (*f <= fold + ALF * alam * slope) {
            return;
        } else {
            if (alam == 1.0) {
                tmplam = -slope / (2.0 * (*f - fold - slope));
            } else {
                rhs1 = *f - fold - alam * slope;
                rhs2 = f2 - fold - alam2 * slope;
                a = (rhs1 / (alam * alam) - rhs2 / (alam2 * alam2)) / (alam - alam2);
                b = (-alam2 * rhs1 / (alam * alam) + alam * rhs2 / (alam2 * alam2)) / (alam - alam2);
                if (a == 0.0) {
                    tmplam = -slope / (2.0 * b);
                } else {
                    disc = b * b - 3.0 * a * slope;
                    if (disc < 0.0) {
                        tmplam = 0.5 * alam;
                    } else if (b <= 0.0) {
                        tmplam = (-b + sqrt(disc)) / (3.0 * a);
                    } else {
                    tmplam = -slope / (b + sqrt(disc));
                    }
                }
                if (tmplam > 0.5 * alam) {
                    tmplam = 0.5 * alam;
                }
            }
        }
        alam2 = alam;
        f2 = *f;
        alam = DMAX(tmplam, 0.1 * alam);
    }
}

void Forces::qrdcmp(double **a, int n, double *c, double *d, int *sing) {
    int i, j, k;
    double scale, sigma, sum, tau;

    *sing = 0;
    for (k = 1; k < n; k++) {
        scale = 0.0;
        for (i = k; i <= n; i++) scale = DMAX(scale, fabs(a[i][k]));
        if (scale == 0.0) {
            *sing = 1;
            c[k] = d[k] = 0.0;
        } else {
            for (i = k; i <= n; i++) a[i][k] /= scale;
            for (sum = 0.0, i = k; i <= n; i++) sum += DSQR(a[i][k]);
            sigma = SIGN(sqrt(sum), a[k][k]);
            a[k][k] += sigma;
            c[k] = sigma * a[k][k];
            d[k] = -scale * sigma;
            for (j = k + 1; j <= n; j++) {
                for (sum = 0.0, i = k; i <= n; i++) sum += a[i][k] * a[i][j];
                tau = sum / c[k];
                for (i = k; i <= n; i++) a[i][j] -= tau * a[i][k];
            }
        }
    }
    d[n] = a[n][n];
    if (d[n] == 0.0) *sing = 1;
}

void Forces::qrupdt(double **r, double **qt, int n, double u[], double v[]) {
    int i, j, k;

    for (k = n; k >= 1; k--) {
        if (u[k]) break;
    }
    if (k < 1) k = 1;
    for (i = k - 1; i >= 1; i--) {
        rotate(r, qt, n, i, u[i], -u[i + 1]);
        if (u[i] == 0.0) {
            u[i] = fabs(u[i + 1]);
        } else if (fabs(u[i]) > fabs(u[i + 1])) {
            u[i] = fabs(u[i]) * sqrt(1.0 + DSQR(u[i + 1] / u[i]));
        } else {
            u[i] = fabs(u[i + 1]) * sqrt(1.0 + DSQR(u[i] / u[i + 1]));
        }
    }
    for (j = 1; j <= n; j++) r[1][j] += u[1] * v[j];
    for (i = 1; i < k; i++)
        rotate(r, qt, n, i, r[i][i], -r[i + 1][i]);
}

void Forces::rotate(double **r, double **qt, int n, int i, double a, double b) {
    int j;
    double c, fact, s, w, y;

    if (a == 0.0) {
        c = 0.0;
        s = (b >= 0.0 ? 1.0 : -1.0);
    } else if (fabs(a) > fabs(b)) {
        fact = b / a;
        c = SIGN(1.0 / sqrt(1.0 + (fact * fact)), a);
        s = fact * c;
    } else {
        fact = a / b;
        s = SIGN(1.0 / sqrt(1.0 + (fact * fact)), b);
        c = fact * s;
    }
    for (j = i; j <= n; j++) {
        y = r[i][j];
        w = r[i + 1][j];
        r[i][j] = c * y - s * w;
        r[i + 1][j] = s * y + c * w;
    }
    for (j = 1; j <= n; j++) {
        y = qt[i][j];
        w = qt[i + 1][j];
        qt[i][j] = c * y - s * w;
        qt[i + 1][j] = s * y + c * w;
    }
}
arma::vec Forces::get_e1_XCG() { return e1_XCG; }
arma::vec Forces::get_e2_XCG() { return e2_XCG; }
arma::vec Forces::get_e3_XCG() { return e3_XCG; }
arma::vec Forces::get_e4_XCG() { return e4_XCG; }
arma::vec Forces::get_FAPB() { return FAPB; }
arma::vec Forces::get_FAP() { return FAP; }
arma::vec Forces::get_FMB() { return FMB; }
arma::vec Forces::get_ddrP_1() { return ddrP_1; }
arma::vec Forces::get_ddang_1() { return ddang_1; }
arma::vec Forces::get_rhoC_1() { return rhoC_1; }
arma::vec Forces::get_ddrhoC_1() { return ddrhoC_1; }
arma::vec Forces::get_SLOSH_CG() { return SLOSH_CG; }
double Forces::get_slosh_mass() { return slosh_mass; }
double Forces::get_ddang_slosh_theta() { return ddang_slosh_theta; }
double Forces::get_ddang_slosh_psi() { return ddang_slosh_psi; }
void Forces::set_reference_point(double refp) { xp = refp; }
void Forces::set_TWD_flag(unsigned int flag) { TWD_flag = flag; }

arma::mat33 Forces::TMX(double ang) {
    arma::mat33 TM;

    TM(0, 0) = 1.0;
    TM(0, 1) = 0.0;
    TM(0, 2) = 0.0;
    TM(1, 0) = 0.0;
    TM(1, 1) = cos(ang);
    TM(1, 2) = sin(ang);
    TM(2, 0) = 0.0;
    TM(2, 1) = -sin(ang);
    TM(2, 2) = cos(ang);

    return TM;
}

arma::mat33 Forces::TMY(double ang) {
    arma::mat33 TM;

    TM(0, 0) = cos(ang);
    TM(0, 1) = 0.0;
    TM(0, 2) = -sin(ang);
    TM(1, 0) = 0.0;
    TM(1, 1) = 1.0;
    TM(1, 2) = 0.0;
    TM(2, 0) = sin(ang);
    TM(2, 1) = 0.0;
    TM(2, 2) = cos(ang);

    return TM;
}

arma::mat33 Forces::TMZ(double ang) {
    arma::mat33 TM;

    TM(0, 0) = cos(ang);
    TM(0, 1) = sin(ang);
    TM(0, 2) = 0.0;
    TM(1, 0) = -sin(ang);
    TM(1, 1) = cos(ang);
    TM(1, 2) = 0.0;
    TM(2, 0) = 0.0;
    TM(2, 1) = 0.0;
    TM(2, 2) = 1.0;

    return TM;
}

#undef MAXITS
#undef EPS
#undef TOLF
#undef TOLMIN
#undef TOLX
#undef STPMX
#undef FREERETURN
#undef NRANSI
