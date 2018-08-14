#ifndef __forces_HH__
#define __forces_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (Describe the forces Module Variables and Algorithm)
LIBRARY DEPENDENCY:
      ((../src/Forces.cpp))
      ((../../math/src/nrutil.cpp))
PROGRAMMERS:
      ((Lai Jun Xu))
*******************************************************************************/
#include "global_constants.hh"
#include "Tvc.hh"
#include "Propulsion.hh"
#include <functional>
#include "aux.hh"
#include <armadillo>

class Propulsion;
class TVC;

class Forces {
    TRICK_INTERFACE(Forces);

 public:
    Forces() {}

    Forces(Propulsion& prop, TVC& tvc);
    Forces(const Forces& other);

    Forces& operator=(const Forces& other);

    void initialize();

    void collect_forces_and_propagate();
    void set_reference_point(double refp);
    void set_Slosh_flag(unsigned int flag);
    void set_TWD_flag(unsigned int flag);
    void set_DOF(int ndof);
    void set_damping_ratio(double damping);
    void set_aero_flag(unsigned int in);
    void set_e1_d(double in1, double in2, double in3);
    void set_e2_d(double in1, double in2, double in3);
    void set_e3_d(double in1, double in2, double in3);
    void set_e4_d(double in1, double in2, double in3);

    std::function<bool()> grab_rcs_isEnabled;
    std::function<int()> grab_rcs_mode;

    arma::vec get_FAPB();
    arma::vec get_FAP();
    arma::vec get_FMB();
    arma::vec get_rhoC_1();
    arma::vec get_ddrhoC_1();
    arma::vec get_ddrP_1();
    arma::vec get_ddang_1();
    arma::vec get_SLOSH_CG();
    double get_ddang_slosh_theta();
    double get_ddang_slosh_psi();
    double get_slosh_mass();
    arma::vec get_e1_XCG();
    arma::vec get_e2_XCG();
    arma::vec get_e3_XCG();
    arma::vec get_e4_XCG();

    std::function<double()> grab_pdynmc;
    std::function<double()> grab_thrust;
    std::function<double()> grab_refa;
    std::function<double()> grab_refd;
    std::function<double()> grab_cy;
    std::function<double()> grab_cll;
    std::function<double()> grab_clm;
    std::function<double()> grab_cln;
    std::function<double()> grab_cx;
    std::function<double()> grab_cz;
    std::function<arma::vec3()> grab_FPB;
    std::function<arma::vec3()> grab_FMPB;
    std::function<arma::vec3()> grab_lx;
    std::function<double()> grab_vmass;
    std::function<arma::vec3()> grab_GRAVG;
    std::function<arma::mat33()> grab_TBI;
    std::function<arma::mat33()> grab_IBBB;
    std::function<arma::vec3()> grab_WBIBD;
    std::function<arma::vec3()> grab_WBIB;
    std::function<arma::vec3()> grab_ABII;
    std::function<arma::vec6()> grab_Q_TVC;
    std::function<arma::vec3()> grab_xcg_0;
    std::function<arma::vec3()> grab_xcp;
    std::function<arma::vec3()> grab_xcg;
    std::function<double()> grab_oxidizer_mass;
    std::function<double()> grab_dang_slosh_theta;
    std::function<double()> grab_ang_slosh_theta;
    std::function<double()> grab_dang_slosh_psi;
    std::function<double()> grab_ang_slosh_psi;
    std::function<arma::vec3()> grab_NEXT_ACC;
    std::function<unsigned int()> grab_liftoff;
    std::function<arma::vec3()> grab_FSPB;
    std::function<double()> grab_dang_e1_B;
    std::function<double()> grab_dang_e2_B;
    std::function<double()> grab_dang_e3_B;
    std::function<double()> grab_dang_e4_B;
    std::function<double()> grab_ang_e1_theta;
    std::function<double()> grab_ang_e2_psi;
    std::function<double()> grab_ang_e3_theta;
    std::function<double()> grab_ang_e4_psi;
    std::function<double()> grab_e1_XCG;
    std::function<double()> grab_e2_XCG;
    std::function<double()> grab_e3_XCG;
    std::function<double()> grab_e4_XCG;
    std::function<double()> grab_e1_mass;
    std::function<double()> grab_e2_mass;
    std::function<double()> grab_e3_mass;
    std::function<double()> grab_e4_mass;
    std::function<arma::mat33()> grab_I_S2_E1;
    std::function<arma::mat33()> grab_I_S2_E2;
    std::function<arma::mat33()> grab_I_S2_E3;
    std::function<arma::mat33()> grab_I_S2_E4;
    std::function<double()> grab_s2_act1_acc;
    std::function<double()> grab_s2_act2_acc;
    std::function<double()> grab_s2_act3_acc;
    std::function<double()> grab_s2_act4_acc;
    std::function<arma::vec3()> grab_structure_XCG;

 private:
    /* Internal Getter */

    /* Internal Initializers */
    void default_data();

    arma::mat33 TMX(double ang);
    arma::mat33 TMY(double ang);
    arma::mat33 TMZ(double ang);
    arma::mat33 cross_matrix(arma::vec3 in);
    void gamma_beta();
    void Gravity_Q();
    void AeroDynamics_Q();
    void calculate_I1();
    void funcv(int n, double *x, double *ff);
    void broydn(double x[], int n, int *check);
    void rsolv(double **a, int n, double d[], double b[]);
    void fdjac(int n, double x[], double fvec[], double **df);
    double f_min(double x[]);
    void lnsrch(int n, double xold[], double fold, double g[], double p[], double x[],
    double *f, double stpmax, int *check);
    void qrdcmp(double **a, int n, double *c, double *d, int *sing);
    void qrupdt(double **r, double **qt, int n, double u[], double v[]);
    void rotate(double **r, double **qt, int n, int i, double a, double b);
    void slosh();
    void S2_TWD();
    void calculate_I_E();

    /* Internal Propagator / Calculators */

    /* Internal Calculators */

    /* Routing references */
    Propulsion   * propulsion;
    TVC          * tvc;

    /* Input */

    /* Constants */

    /* Propagative Stats */

    /* Generating Outputs */
    arma::vec FAPB;         /* *o (N)      Aerodynamic and propulsion forces in body axes */
    double _FAPB[3];        /* *o (N)      Aerodynamic and propulsion forces in body axes */

    arma::vec FAP;          /* *o (N)      Aerodynamic force in body axes */
    double _FAP[3];         /* *o (N)      Aerodynamic force in body axes */

    arma::vec FMB;          /* *o (N*m)    Aerodynamic and propulsion moment in body axes */
    double _FMB[3];         /* *o (N*m)    Aerodynamic and propulsion moment in body axes */

    arma::vec FMAB;          /* *o (N*m)    Aerodynamic and propulsion moment in body axes */
    double _FMAB[3];         /* *o (N*m)    Aerodynamic and propulsion moment in body axes */

    arma::vec Q_G;
    double _Q_G[8];

    arma::vec Q_Aero;
    double _Q_Aero[6];

    arma::vec rhoC_1;
    double _rhoC_1[3];

    arma::mat I1;
    double _I1[3][3];

    arma::vec ddrP_1;
    double _ddrP_1[3];

    arma::vec ddang_1;
    double _ddang_1[3];

    arma::vec dang_1;
    double _dang_1[3];

    arma::vec ddrhoC_1;
    double _ddrhoC_1[3];

    arma::vec p_b1_ga;
    double _p_b1_ga[3];

    arma::vec p_b1_be;
    double _p_b1_be[3];

    arma::vec f;
    double _f[12];

    arma::vec gamma_b1_q1;
    double _gamma_b1_q1[3];

    arma::vec gamma_b1_q2;
    double _gamma_b1_q2[3];

    arma::vec gamma_b1_q3;
    double _gamma_b1_q3[3];

    arma::vec gamma_b1_q4;
    double _gamma_b1_q4[3];

    arma::vec gamma_b1_q5;
    double _gamma_b1_q5[3];

    arma::vec gamma_b1_q6;
    double _gamma_b1_q6[3];

    arma::vec beta_b1_q1;
    double _beta_b1_q1[3];

    arma::vec beta_b1_q2;
    double _beta_b1_q2[3];

    arma::vec beta_b1_q3;
    double _beta_b1_q3[3];

    arma::vec beta_b1_q4;
    double _beta_b1_q4[3];

    arma::vec beta_b1_q5;
    double _beta_b1_q5[3];

    arma::vec beta_b1_q6;
    double _beta_b1_q6[3];

    arma::vec beta_slosh_q4;
    double _beta_slosh_q4[3];

    arma::vec beta_slosh_q5;
    double _beta_slosh_q5[3];

    arma::vec beta_slosh_q6;
    double _beta_slosh_q6[3];

    arma::vec beta_slosh_q7;
    double _beta_slosh_q7[3];

    arma::vec beta_slosh_q8;
    double _beta_slosh_q8[3];

    arma::vec beta_S2_e1_q4;
    double _beta_S2_e1_q4[3];

    arma::vec beta_S2_e1_q5;
    double _beta_S2_e1_q5[3];

    arma::vec beta_S2_e1_q6;
    double _beta_S2_e1_q6[3];

    arma::vec beta_S2_e1_q_theta;
    double _beta_S2_e1_q_theta[3];

    arma::vec beta_S2_e2_q4;
    double _beta_S2_e2_q4[3];

    arma::vec beta_S2_e2_q5;
    double _beta_S2_e2_q5[3];

    arma::vec beta_S2_e2_q6;
    double _beta_S2_e2_q6[3];

    arma::vec beta_S2_e2_q_psi;
    double _beta_S2_e2_q_psi[3];

    arma::vec beta_S2_e3_q4;
    double _beta_S2_e3_q4[3];

    arma::vec beta_S2_e3_q5;
    double _beta_S2_e3_q5[3];

    arma::vec beta_S2_e3_q6;
    double _beta_S2_e3_q6[3];

    arma::vec beta_S2_e3_q_theta;
    double _beta_S2_e3_q_theta[3];

    arma::vec beta_S2_e4_q4;
    double _beta_S2_e4_q4[3];

    arma::vec beta_S2_e4_q5;
    double _beta_S2_e4_q5[3];

    arma::vec beta_S2_e4_q6;
    double _beta_S2_e4_q6[3];

    arma::vec beta_S2_e4_q_psi;
    double _beta_S2_e4_q_psi[3];

    arma::vec ddrP_e1;
    double _ddrP_e1[3];

    arma::vec ddrP_e2;
    double _ddrP_e2[3];

    arma::vec ddrP_e3;
    double _ddrP_e3[3];

    arma::vec ddrP_e4;
    double _ddrP_e4[3];

    arma::vec ddrhoC_e1;
    double _ddrhoC_e1[3];

    arma::vec ddrhoC_e2;
    double _ddrhoC_e2[3];

    arma::vec ddrhoC_e3;
    double _ddrhoC_e3[3];

    arma::vec ddrhoC_e4;
    double _ddrhoC_e4[3];

    double xp;

    double slosh_mass;
    double pendulum_L;
    double ddang_slosh_theta;
    double ddang_slosh_psi;
    double Q_slosh1;
    double Q_slosh2;
    double ddang_e1_theta;
    double ddang_e2_psi;
    double ddang_e3_theta;
    double ddang_e4_psi;
    double Q_e1;
    double Q_e2;
    double Q_e3;
    double Q_e4;

    arma::vec f_slosh;
    double _f_slosh[8];

    arma::vec f_S2_TWD;
    double _f_S2_TWD[10];

    arma::vec ddang_slosh;
    double _ddang_slosh[3];

    arma::vec ddrP_slosh;
    double _ddrP_slosh[3];

    arma::vec ddrP_sloshB;
    double _ddrP_sloshB[3];

    arma::vec ddrhoC_slosh;
    double _ddrhoC_slosh[3];

    arma::vec rhoC_slosh;
    double _rhoC_slosh[3];

    arma::vec r_slosh;
    double _r_slosh[3];

    arma::vec Q_G_slosh;
    double _Q_G_slosh[8];

    arma::vec Q_G_S2_E;
    double _Q_G_S2_E[10];

    double Q1;
    double Q2;

    arma::mat TBSLOSH_I;
    double _TBSLOSH_I[3][3];

    arma::vec dang_slosh1;
    double _dang_slosh1[3];

    arma::vec dang_slosh2;
    double _dang_slosh2[3];

    arma::vec omega_slosh1_B;
    double _omega_slosh1_B[3];

    arma::vec omega_slosh2_slosh1;
    double _omega_slosh2_slosh1[3];

    arma::mat TSLOSHB1_B;
    double _TSLOSHB1_B[3][3];

    arma::mat TSLOSHB2_SLOSHB1;
    double _TSLOSHB2_SLOSHB1[3][3];

    arma::vec SLOSH_CG;
    double _SLOSH_CG[3];

    arma::mat TE1_B;
    double _TE1_B[3][3];

    arma::mat TE2_B;
    double _TE2_B[3][3];

    arma::mat TE3_B;
    double _TE3_B[3][3];

    arma::mat TE4_B;
    double _TE4_B[3][3];

    arma::mat TE1_I;
    double _TE1_I[3][3];

    arma::mat TE2_I;
    double _TE2_I[3][3];

    arma::mat TE3_I;
    double _TE3_I[3][3];

    arma::mat TE4_I;
    double _TE4_I[3][3];

    arma::vec omega_e1_B;
    double _omega_e1_B[3];

    arma::vec omega_e2_B;
    double _omega_e2_B[3];

    arma::vec omega_e3_B;
    double _omega_e3_B[3];

    arma::vec omega_e4_B;
    double _omega_e4_B[3];

    arma::vec domega_e1;
    double _domega_e1[3];

    arma::vec domega_e2;
    double _domega_e2[3];

    arma::vec domega_e3;
    double _domega_e3[3];

    arma::vec domega_e4;
    double _domega_e4[3];

    arma::vec e1_d;
    double _e1_d[3];

    arma::vec e2_d;
    double _e2_d[3];

    arma::vec e3_d;
    double _e3_d[3];

    arma::vec e4_d;
    double _e4_d[3];

    arma::vec dang_e1;
    double _dang_e1[3];

    arma::vec dang_e2;
    double _dang_e2[3];

    arma::vec dang_e3;
    double _dang_e3[3];

    arma::vec dang_e4;
    double _dang_e4[3];

    arma::vec rhoC_e1;
    double _rhoC_e1[3];

    arma::vec rhoC_e2;
    double _rhoC_e2[3];

    arma::vec rhoC_e3;
    double _rhoC_e3[3];

    arma::vec rhoC_e4;
    double _rhoC_e4[3];

    arma::mat I_E1;
    double _I_E1[3][3];

    arma::mat I_E2;
    double _I_E2[3][3];

    arma::mat I_E3;
    double _I_E3[3][3];

    arma::mat I_E4;
    double _I_E4[3][3];

    arma::vec e1_XCG;
    double _e1_XCG[3];

    arma::vec e2_XCG;
    double _e2_XCG[3];

    arma::vec e3_XCG;
    double _e3_XCG[3];

    arma::vec e4_XCG;
    double _e4_XCG[3];

    int its;
    int DOF;
    unsigned int Slosh_flag;
    unsigned int TWD_flag;
    unsigned int Aero_flag;
    double h;
    double Wn;
    double C_c;
    double C_1;
    double damping_ratio;

    double Q_E1;
    double Q_E2;
    double Q_E3;
    double Q_E4;
    double Q_slosh_7;
    double Q_slosh_8;
};

#endif  // __forces_HH__
