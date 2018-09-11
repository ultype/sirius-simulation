#include "Control_c.h"



void calculate_xcg_thrust(const double int_step, const double fmass0, const double mdot, const double xcg_0, const double xcg_1
                            , const double isp, double *fmasse, double *mass_ratio, double *xcg, double *thrust) {
    *fmasse += mdot * int_step;
    *mass_ratio = (*fmasse) / fmass0;
    *xcg = xcg_0 + (xcg_1 - xcg_0) * (*mass_ratio);
    *thrust = isp * mdot * __AGRAV / 4.0;
    return;
}

void S3_B_pseudo_G(gsl_vector *cmd, gsl_vector *IBBB0, gsl_vector *IBBB1, gsl_vector *IBBB2, double *lx
                    , double *theta_a_cmd, double *theta_b_cmd, const double xcg, const double reference_point
                    , const double thrust, const double mass_ratio, const double int_step) {
    gsl_vector *tmp, *anglecmd;
    gsl_matrix *G, *B_pseudo;

    B_pseudo = gsl_matrix_calloc(2, 3);
    G = gsl_matrix_calloc(3, 3);
    tmp = gsl_vector_calloc(3);
    anglecmd = gsl_vector_calloc(3);

    *lx = -xcg - reference_point;
    
    gsl_matrix_set(B_pseudo, 0, 0, 0.0);
    gsl_matrix_set(B_pseudo, 0, 1, 0.0);
    gsl_matrix_set(B_pseudo, 0, 2, -1.0);
    gsl_matrix_set(B_pseudo, 1, 0, 0.0);
    gsl_matrix_set(B_pseudo, 1, 1, -1.0);
    gsl_matrix_set(B_pseudo, 1, 2, 0.0);

    gsl_vector_memcpy(tmp, IBBB1);
    gsl_vector_sub(tmp, IBBB0);  // tmp = IBBB1 - IBBB0
    gsl_vector_scale(tmp, mass_ratio);  // tmp * mass_ratio
    gsl_vector_add(tmp ,IBBB0);  // tmp + IBBB0
    gsl_vector_memcpy(IBBB2, tmp);  // IBBB2 = IBBB0 + (IBBB1 - IBBB0) * mass_ratio

    gsl_matrix_set(G, 0, 0, 0.0);
    gsl_matrix_set(G, 1, 1, (gsl_vector_get(IBBB2, 1) / (thrust * 4.0 * (*lx))));
    gsl_matrix_set(G, 2, 2, (gsl_vector_get(IBBB2, 2) / (thrust * 4.0 * (*lx))));

    gsl_blas_dgemv(CblasNoTrans, 1.0, G, cmd, 0.0, tmp);  // tmp = G * cmd
    gsl_blas_dgemv(CblasNoTrans, 1.0, B_pseudo, tmp, 0.0, anglecmd);  // anglecmd = B_pseudo * tmp

    *theta_a_cmd = gsl_vector_get(anglecmd, 0);
    *theta_b_cmd = gsl_vector_get(anglecmd, 1);

    gsl_matrix_free(B_pseudo);
    gsl_matrix_free(G);
    gsl_vector_free(tmp);
    gsl_vector_free(anglecmd);

    return;
}

void S2_B_pseudo_G(gsl_vector *cmd, gsl_vector *IBBB0, gsl_vector *IBBB1, gsl_vector *IBBB2
                    , double *lx, double *theta_a_cmd, double *theta_b_cmd, double *theta_c_cmd
                    , double *theta_d_cmd, const double d, const double xcg, const double reference_point
                    , const double thrust, const double mass_ratio, const double int_step) {
    gsl_vector *tmp, *anglecmd, *CMDG;
    gsl_matrix *G, *B_pseudo;

    B_pseudo = gsl_matrix_calloc(3, 4);
    G = gsl_matrix_calloc(3, 3);
    tmp = gsl_vector_calloc(3);
    anglecmd = gsl_vector_calloc(4);
    CMDG = gsl_vector_calloc(3);

    *lx = -xcg - (reference_point);
    
    gsl_matrix_set(B_pseudo, 0, 0, 0.5 * thrust * d);
    gsl_matrix_set(B_pseudo, 0, 1, -0.5 * thrust * d);
    gsl_matrix_set(B_pseudo, 0, 2, -0.5 * thrust * d);
    gsl_matrix_set(B_pseudo, 0, 3, 0.5 * thrust * d);
    gsl_matrix_set(B_pseudo, 1, 0, 0.0);
    gsl_matrix_set(B_pseudo, 1, 1, 0.0);
    gsl_matrix_set(B_pseudo, 1, 2, -1.0 * thrust * *lx);
    gsl_matrix_set(B_pseudo, 1, 3, -1.0 * thrust * *lx);
    gsl_matrix_set(B_pseudo, 2, 0, -1.0 * thrust * *lx);
    gsl_matrix_set(B_pseudo, 2, 1, -1.0 * thrust * *lx);
    gsl_matrix_set(B_pseudo, 2, 2, 0.0);
    gsl_matrix_set(B_pseudo, 2, 3, 0.0);

    gsl_vector_memcpy(tmp, IBBB1);
    gsl_vector_sub(tmp, IBBB0);  // tmp = IBBB1 - IBBB0
    gsl_vector_scale(tmp, mass_ratio);  // tmp * mass_ratio
    gsl_vector_add(tmp, IBBB0);  // tmp + IBBB0
    gsl_vector_memcpy(IBBB2, tmp);  // IBBB2 = IBBB0 + (IBBB1 - IBBB0) * mass_ratio

    gsl_matrix_set(G, 0, 0, gsl_vector_get(IBBB2, 0));
    gsl_matrix_set(G, 1, 1, gsl_vector_get(IBBB2, 1));
    gsl_matrix_set(G, 2, 2, gsl_vector_get(IBBB2, 2));

    gsl_blas_dgemv(CblasNoTrans, 1.0, G, cmd, 0.0, CMDG);  // CMDG = G * cmd
    gsl_blas_dgemv(CblasNoTrans, 1.0, moore_penrose_pinv(B_pseudo, 1e-15), CMDG, 0.0, anglecmd);  // B_pseudo1 = pinv(B_pseudo); anglecmd = B_pseudo1 * CMDG

    *theta_a_cmd = gsl_vector_get(anglecmd, 0);
    *theta_b_cmd = gsl_vector_get(anglecmd, 1);
    *theta_c_cmd = gsl_vector_get(anglecmd, 2);
    *theta_d_cmd = gsl_vector_get(anglecmd, 3);

    return;
}

void Quaternion_cmd(const double int_step, double pitchcmd, double pitchcmd_old, double pitchcmd_out_old
                    , double pitchcmd_new, double rollcmd, double yawcmd, gsl_vector *TCMDQ, gsl_vector *CMDQ
                    , gsl_matrix *TLI, gsl_matrix *TBIC) {
    gsl_matrix *TBL, *TCMD;
    gsl_vector *TBLQ, *ANGLECMD, *TDBQ;
    TBL = gsl_matrix_calloc(3, 3);
    TBLQ = gsl_vector_calloc(4);
    ANGLECMD = gsl_vector_calloc(3);
    TCMD = gsl_matrix_calloc(3, 3);
    TDBQ = gsl_vector_calloc(4);

    gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, TBIC, TLI, 0.0, TBL);

    // Pitch down program : 1st order low pass filter
    pitchcmd_new = 0.02439 * pitchcmd + 0.02439 * pitchcmd_old + 0.9512 * pitchcmd_out_old;
    pitchcmd_old = pitchcmd;
    pitchcmd_out_old = pitchcmd_new;

    gsl_vector_set(ANGLECMD, 0, rollcmd * __RAD);
    gsl_vector_set(ANGLECMD, 1, pitchcmd_new * __RAD);
    gsl_vector_set(ANGLECMD, 2, yawcmd * __RAD);

    build_321_rotation_matrix(ANGLECMD, TCMD);
    Matrix2Quaternion_C(TCMD, TCMDQ);
    Matrix2Quaternion_C(TBL, TBLQ);

    TDBQ = Quaternion_conjugate_C(TBLQ);
    Quaternion_cross_C(TDBQ, TCMDQ, CMDQ);

    gsl_vector_scale(CMDQ, sign(gsl_vector_get(CMDQ, 0)));

    gsl_matrix_free(TBL);
    gsl_vector_free(TBLQ);
    gsl_vector_free(ANGLECMD);
    gsl_matrix_free(TCMD);
    gsl_vector_free(TDBQ);

    return;
}

void pitch_down(const double int_step, double pitchcmd, double kpp, double kpi
                , double kpd, double pN, double perror_old, double pitchiout_old
                ,double pdout_old, gsl_vector *WBICB, gsl_vector *CONTROLCMD) {
    thterror = 2.0 * pitchcmd;
    double p1out = thterror * kpp;
    perrori = 0.5 * (kpi * int_step * perror_old + 2.0 * pitchiout_old + kpi * int_step * thterror);
    double dout = (2.0 * kpd * pN * thterror - 2.0 * kpd * pN * perror_old - (int_step * pN - 2.0) * pdout_old) / (2.0 + int_step * pN);
    double iout = perrori;
    perrorp = p1out + iout + dout - gsl_vector_get(WBICB, 1);
    double p2out = perrorp * kppp;

    gsl_vector_set(CONTROLCMD, 1, p2out);

    perror_old = thterror;
    pitchiout_old = iout;
    pdout_old = dout;

    return;
}

void roll_control(const double int_step, double rollcmd, double krp, double kri, double krd
                    , double rN, double rollerror, double rerror_old, double rolliout_old
                    , double rdout_old, gsl_vector *WBICB, gsl_vector *CONTROLCMD) {
    rollerror = 2.0 * rollcmd;
    double p1out = rollerror * krp;
    rerrori = 0.5 * (kri * int_step * rerror_old + 2.0 * rolliout_old + kri * int_step * rollerror);
    double dout = (2.0 * krd * rN * rollerror - 2.0 * krd * rN * rerror_old - (int_step * rN - 2.0) * rdout_old) / (2.0 + int_step * rN);
    double iout = rerrori;
    rerrorp = p1out + iout + dout - gsl_vector_get(WBICB, 0);
    double p2out = rerrorp * krpp;

    gsl_vector_set(CONTROLCMD, 0, p2out);

    rerror_old = rollerror;
    rolliout_old = iout;
    rdout_old = dout;

    return;
}

void yaw_control(const double int_step, double yawcmd, double kyp, double kyi, double kyd
                    , double yN, double yawerror, double yerror_old, double yawiout_old
                    , double ydout_old, gsl_vector *WBICB, gsl_vector *CONTROLCMD) {
    yawerror = 2.0 * yawcmd;
    double p1out = yawerror * kyp;
    yerrori = 0.5 * (kyi * int_step * yerror_old + 2.0 * yawiout_old + kyi * int_step * yawerror);
    double dout = (2.0 * kyd * yN * yawerror - 2.0 * kyd * yN * yerror_old - (int_step * yN - 2.0) * ydout_old) / (2.0 + int_step * yN);
    double iout = yerrori;
    yerrorp = p1out + iout + dout - gsl_vector_get(WBICB, 2);
    double p2out = yerrorp * kypp;

    gsl_vector_set(CONTROLCMD, 2, p2out);

    yerror_old = yawerror;
    yawiout_old = iout;
    ydout_old = dout;

    return;
}

void AOA_control(const double int_step, double aoacmd, double kaoap, double kaoai, double kaoad
                    , double alphacx, double aoaerror, double aoaerror_old, double aoaiout_old
                    , double aoadout_old, gsl_vector *WBICB, gsl_vector *CONTROLCMD) {
    aoaerror = aoacmd - 0.5 * alphacx * __RAD;

    // aoaerror_new = 0.02439 * aoaerror + 0.02439 * aoaerror_smooth_old + 0.9512 * aoaerror_out_old;
    // aoaerror_smooth_old = aoaerror;
    // aoaerror_out_old = aoaerror_new;
    double aoaerror_new = aoaerror;

    double p1out = aoaerror_new * kaoap;
    aoaerrori = 0.5 * (kaoai * int_step * aoaerror_old + 2.0 * aoaiout_old + kaoai * int_step * aoaerror_new);
    // double dout = (2.0 * kaoad * aoaN * aoaerror_new - 2.0 * kaoad * aoaN * aoaerror_old - (int_step * aoaN - 2.0) * aoadout_old) / (2.0 + int_step * aoaN);
    double dout = kaoad / int_step * (aoaerror_new - aoaerror_old);
    double iout = aoaerrori;
    aoaerrorp = (p1out + iout + dout) - gsl_vector_get(WBICB, 1);
    double p2out = aoaerrorp * kaoapp;

    gsl_vector_set(CONTROLCMD, 1, p2out);

    aoaerror_old = aoaerror_new;
    aoaiout_old = iout;
    aoadout_old = dout;

    return;
}

void control(const double int_step, const double altc, gsl_matrix *TLI, gsl_matrix *TBIC
            , const int mode, gsl_vector *delta_euler) {
    gsl_matrix *TBL;
    gsl_vector *euler;
    TBL = gsl_matrix_calloc(3, 3);
    euler = gsl_vector_calloc(3);
    euler_angle(TBL, euler);
    switch(mode) {
        case 0:
            return;
            break;

        case 1:
            calculate_xcg_thrust(int_step, fmass0, mdot, xcg_0, xcg_1
                                , isp, &fmasse, &mass_ratio, &xcg, &thrust);
            pitchcmd_new = 0.02439 * pitchcmd + 0.02439 * pitchcmd_old + 0.9512 * pitchcmd_out_old;
            pitchcmd_old = pitchcmd;
            pitchcmd_out_old = pitchcmd_new;
            gsl_vector_set(delta_euler, 0, rollcmd * __RAD - gsl_vector_get(euler, 0));
            gsl_vector_set(delta_euler, 1, pitchcmd_new * __RAD  - gsl_vector_get(euler, 1));
            gsl_vector_set(delta_euler, 2, yawcmd * __RAD - gsl_vector_get(euler, 2));
            break;




        default:
            break;
    }


}





