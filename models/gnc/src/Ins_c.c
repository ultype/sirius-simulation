#include "Ins_c.h"


/********************************************************************
*
* Earth gravity field JGM3
* Gravitational coefficients C, S are efficiently stored in a single
* array CS. The lower triangle matrix CS holds the non-sectorial C
* coefficients C_n,m (n!=m). Sectorial C coefficients C_n,n are the
* diagonal elements of CS and the upper triangular matrix stores
* the S_n,m (m!=0) coefficients in columns, for the same degree n.
* Mapping of CS to C, S is achieved through
* C_n,m = CS(n,m), S_n,m = CS(m-1,n)
*
*********************************************************************/

#define N_JGM3  20
#define Max_DM_UT1_UT_Index 954

extern const double DM_UT1_UT[Max_DM_UT1_UT_Index];

static double INS_CS_JGM3[N_JGM3+1][N_JGM3+1] = {
    { 1.000000e+00,  0.000000e+00,  1.543100e-09,  2.680119e-07, -4.494599e-07,
     -8.066346e-08,  2.116466e-08,  6.936989e-08,  4.019978e-08,  1.423657e-08,
     -8.128915e-08, -1.646546e-08, -2.378448e-08,  2.172109e-08,  1.443750e-08,
      4.154186e-09,  1.660440e-08, -1.427822e-08, -1.817656e-08,  7.160542e-11,
      2.759192e-09                                                           },
    { 0.000000e+00,  0.000000e+00, -9.038681e-07, -2.114024e-07,  1.481555e-07,
     -5.232672e-08, -4.650395e-08,  9.282314e-09,  5.381316e-09, -2.228679e-09,
     -3.057129e-09, -5.097360e-09,  1.416422e-09, -2.545587e-09, -1.089217e-10,
     -1.045474e-09,  7.856272e-10,  2.522818e-10,  3.427413e-10, -1.008909e-10,
      3.216826e-10                                                           },
    {-1.082627e-03, -2.414000e-10,  1.574536e-06,  1.972013e-07, -1.201129e-08,
     -7.100877e-09,  1.843134e-10, -3.061150e-09, -8.723520e-10, -5.633921e-10,
     -8.989333e-10, -6.863521e-10,  9.154575e-11,  3.005522e-10,  5.182512e-11,
      3.265044e-11, -4.271981e-11,  1.297841e-11, -4.278803e-12, -1.190759e-12,
      3.778260e-11                                                           },
    { 2.532435e-06,  2.192799e-06,  3.090160e-07,  1.005589e-07,  6.525606e-09,
      3.873005e-10, -1.784491e-09, -2.636182e-10,  9.117736e-11,  1.717309e-11,
     -4.622483e-11, -2.677798e-11,  9.170517e-13, -2.960682e-12, -3.750977e-12,
      1.116419e-12,  5.250141e-12,  2.159727e-12,  1.105860e-13, -3.556436e-13,
     -1.178441e-12                                                           },
    { 1.619331e-06, -5.087253e-07,  7.841223e-08,  5.921574e-08, -3.982396e-09,
     -1.648204e-09, -4.329182e-10,  6.397253e-12,  1.612521e-11, -5.550919e-12,
     -3.122269e-12,  1.982505e-12,  2.033249e-13,  1.214266e-12, -2.217440e-13,
      8.637823e-14, -1.205563e-14,  2.923804e-14,  1.040715e-13,  9.006136e-14,
     -1.823414e-14                                                           },
    { 2.277161e-07, -5.371651e-08,  1.055905e-07, -1.492615e-08, -2.297912e-09,
      4.304768e-10, -5.527712e-11,  1.053488e-11,  8.627743e-12,  2.940313e-12,
     -5.515591e-13,  1.346234e-13,  9.335408e-14, -9.061871e-15,  2.365713e-15,
     -2.505252e-14, -1.590014e-14, -9.295650e-15, -3.743268e-15,  3.176649e-15,
     -5.637288e-17                                                           },
    {-5.396485e-07, -5.987798e-08,  6.012099e-09,  1.182266e-09, -3.264139e-10,
     -2.155771e-10,  2.213693e-12,  4.475983e-13,  3.814766e-13, -1.846792e-13,
     -2.650681e-15, -3.728037e-14,  7.899913e-15, -9.747983e-16, -3.193839e-16,
      2.856094e-16, -2.590259e-16, -1.190467e-16,  8.666599e-17, -8.340023e-17,
     -8.899420e-19                                                           },
    { 3.513684e-07,  2.051487e-07,  3.284490e-08,  3.528541e-09, -5.851195e-10,
      5.818486e-13, -2.490718e-11,  2.559078e-14,  1.535338e-13, -9.856184e-16,
     -1.052843e-14,  1.170448e-15,  3.701523e-16, -1.095673e-16, -9.074974e-17,
      7.742869e-17,  1.086771e-17,  4.812890e-18,  2.015619e-18, -5.594661e-18,
      1.459810e-18                                                           },
    { 2.025187e-07,  1.603459e-08,  6.576542e-09, -1.946358e-10, -3.189358e-10,
     -4.615173e-12, -1.839364e-12,  3.429762e-13, -1.580332e-13,  7.441039e-15,
     -7.011948e-16,  2.585245e-16,  6.136644e-17,  4.870630e-17,  1.489060e-17,
      1.015964e-17, -5.700075e-18, -2.391386e-18,  1.794927e-18,  1.965726e-19,
     -1.128428e-19                                                           },
    { 1.193687e-07,  9.241927e-08,  1.566874e-09, -1.217275e-09, -7.018561e-12,
     -1.669737e-12,  8.296725e-13, -2.251973e-13,  6.144394e-14, -3.676763e-15,
     -9.892610e-17, -1.736649e-17,  9.242424e-18, -4.153238e-18, -6.937464e-20,
      3.275583e-19,  1.309613e-19,  1.026767e-19, -1.437566e-20, -1.268576e-20,
     -6.100911e-21                                                           },
    { 2.480569e-07,  5.175579e-08, -5.562846e-09, -4.195999e-11, -4.967025e-11,
     -3.074283e-12, -2.597232e-13,  6.909154e-15,  4.635314e-15,  2.330148e-15,
      4.170802e-16, -1.407856e-17, -2.790078e-19, -6.376262e-20, -1.849098e-19,
      3.595115e-20, -2.537013e-21,  4.480853e-21,  4.348241e-22,  1.197796e-21,
     -1.138734e-21                                                           },
    {-2.405652e-07,  9.508428e-09,  9.542030e-10, -1.409608e-10, -1.685257e-11,
      1.489441e-12, -5.754671e-15,  1.954262e-15, -2.924949e-16, -1.934320e-16,
     -4.946396e-17,  9.351706e-18, -9.838299e-20,  1.643922e-19, -1.658377e-20,
      2.905537e-21,  4.983891e-22,  6.393876e-22, -2.294907e-22,  6.437043e-23,
      6.435154e-23                                                           },
    { 1.819117e-07, -3.068001e-08,  6.380398e-10,  1.451918e-10, -2.123815e-11,
      8.279902e-13,  7.883091e-15, -4.131557e-15, -5.708254e-16,  1.012728e-16,
     -1.840173e-18,  4.978700e-19, -2.108949e-20,  2.503221e-20,  3.298844e-21,
     -8.660491e-23,  6.651727e-24,  5.110031e-23, -3.635064e-23, -1.311958e-23,
      1.534228e-24                                                           },
    { 2.075677e-07, -2.885131e-08,  2.275183e-09, -6.676768e-11, -3.452537e-13,
      1.074251e-12, -5.281862e-14,  3.421269e-16, -1.113494e-16,  2.658019e-17,
      4.577888e-18, -5.902637e-19, -5.860603e-20, -2.239852e-20, -6.914977e-23,
     -6.472496e-23, -2.741331e-23,  2.570941e-24, -1.074458e-24, -4.305386e-25,
     -2.046569e-25                                                           },
    {-1.174174e-07, -9.997710e-09, -1.347496e-09,  9.391106e-11,  3.104170e-13,
      3.932888e-13, -1.902110e-14,  2.787457e-15, -2.125248e-16,  1.679922e-17,
      1.839624e-18,  7.273780e-20,  4.561174e-21,  2.347631e-21, -7.142240e-22,
     -2.274403e-24, -2.929523e-24,  1.242605e-25, -1.447976e-25, -3.551992e-26,
     -7.473051e-28                                                           },
    { 1.762727e-08,  6.108862e-09, -7.164511e-10,  1.128627e-10, -6.013879e-12,
      1.293499e-13,  2.220625e-14,  2.825477e-15, -1.112172e-16,  3.494173e-18,
      2.258283e-19, -1.828153e-21, -6.049406e-21, -5.705023e-22,  1.404654e-23,
     -9.295855e-24,  5.687404e-26,  1.057368e-26,  4.931703e-27, -1.480665e-27,
      2.400400e-29                                                           },
    {-3.119431e-08,  1.356279e-08, -6.713707e-10, -6.451812e-11,  4.698674e-12,
     -9.690791e-14,  6.610666e-15, -2.378057e-16, -4.460480e-17, -3.335458e-18,
     -1.316568e-19,  1.643081e-20,  1.419788e-21,  9.260416e-23, -1.349210e-23,
     -1.295522e-24, -5.943715e-25, -9.608698e-27,  3.816913e-28, -3.102988e-28,
     -8.192994e-29                                                           },
    { 1.071306e-07, -1.262144e-08, -4.767231e-10,  1.175560e-11,  6.946241e-13,
     -9.316733e-14, -4.427290e-15,  4.858365e-16,  4.814810e-17,  2.752709e-19,
     -2.449926e-20, -6.393665e-21,  8.842755e-22,  4.178428e-23, -3.177778e-24,
      1.229862e-25, -8.535124e-26, -1.658684e-26, -1.524672e-28, -2.246909e-29,
     -5.508346e-31                                                           },
    { 4.421672e-08,  1.958333e-09,  3.236166e-10, -5.174199e-12,  4.022242e-12,
      3.088082e-14,  3.197551e-15,  9.009281e-17,  2.534982e-17, -9.526323e-19,
      1.741250e-20, -1.569624e-21, -4.195542e-22, -6.629972e-24, -6.574751e-25,
     -2.898577e-25,  7.555273e-27,  3.046776e-28,  3.696154e-29,  1.845778e-30,
      6.948820e-31                                                           },
    {-2.197334e-08, -3.156695e-09,  7.325272e-10, -1.192913e-11,  9.941288e-13,
      3.991921e-14, -4.220405e-16,  7.091584e-17,  1.660451e-17,  9.233532e-20,
     -5.971908e-20,  1.750987e-21, -2.066463e-23, -3.440194e-24, -1.487095e-25,
     -4.491878e-26, -4.558801e-27,  5.960375e-28,  8.263952e-29, -9.155723e-31,
     -1.237749e-31                                                           },
    { 1.203146e-07,  3.688524e-09,  4.328972e-10, -6.303973e-12,  2.869669e-13,
     -3.011115e-14,  1.539793e-15, -1.390222e-16,  1.766707e-18,  3.471731e-19,
     -3.447438e-20,  8.760347e-22, -2.271884e-23,  5.960951e-24,  1.682025e-25,
     -2.520877e-26, -8.774566e-28,  2.651434e-29,  8.352807e-30, -1.878413e-31,
      4.054696e-32                                                           }
};

int calculate_INS_derived_TEI(GPS_TIME gps, gsl_matrix *TEIC) {
  UTC_TIME utc_caldate;
  gsl_matrix *M_rotation, *M_nutation, *M_precession, *M_nut_n_pre;
  double UTC, UT1, dUT1;
  double t, t2, t3, thetaA, zetaA, zA;
  double s_thetaA, c_thetaA, s_zetaA, c_zetaA, s_zA, c_zA;
  double epsilonA = 0.0;
  double temps_sideral = 0.0;
  double delta_psi = 0.0;
  double mjd;
  int index;
  M_rotation = gsl_matrix_calloc(3, 3);
  M_nutation = gsl_matrix_calloc(3, 3);
  M_precession = gsl_matrix_calloc(3, 3);
  M_nut_n_pre = gsl_matrix_calloc(3, 3);
  gsl_matrix_set_identity(M_nutation);
  /*------------------------------------------------------------------ */
    /* --------------- Interface to Global Variable ------------*/
    /*------------------------------------------------------------------ */

    /*------------------------------------------------------------- */
    /*--------------- Calculate the UTC time -------------*/
    /*------------------------------------------------------------- */
    /* GPS time converted from GPS format to YYYY/MM/DD/MM/SS */
    /* Correction for time difference btwn GPS & UTC is applied implicitly */
    /***to prevent change origin gpstime data****/
  GPS_TIME_2_UTC_TIME(gps, &utc_caldate);
  UTC = utc_caldate.hour * 3600.0 + utc_caldate.min * 60.0 + utc_caldate.sec;
  GPS_TIME_2_Modified_julian_date(gps, &mjd);

  index = (int)(mjd - 55197.00);  /* get dUT1 = Ut1 - UT from table*/
  if ((index >= 0) && (index < Max_DM_UT1_UT_Index)) {    /*MJD = 55197.00 (1-1-2010)~ 56150.00(8-11-2012) */
        dUT1 = DM_UT1_UT[index];
    } else {
        dUT1 = -0.008853655954360;  /* mean value during 19760519~20120811, FSW: dUT1 = 0.4; */
    }

    UT1 = UTC + dUT1;

    /*----------------------------------------------------------- */
    /*-------------- Precession Matrix  ------------------- */
    /*----------------------------------------------------------- */
    double jd;
    MJD_2_JD(mjd, &jd);
    t = (jd - 2451545.0) / 36525.0;  /* J2000.5 : Julian Day is 2451545, unit in day */

    t2 = t * t;
    t3 = t2 * t;
    
    thetaA = 2004.3109 * t - 0.42665 * t2 - 0.041833 * t3; /* unit : arcsec */
    zetaA  = 2306.2181 * t + 0.30188 * t2 + 0.017998 * t3; /* unit : arcsec */
    zA     = 2306.2181 * t + 1.09468 * t2 + 0.018203 * t3; /* unit : arcsec */

    s_thetaA = sin(thetaA * __DM_arcsec2r);
    c_thetaA = cos(thetaA * __DM_arcsec2r);
    s_zetaA  = sin(zetaA  * __DM_arcsec2r);
    c_zetaA  = cos(zetaA  * __DM_arcsec2r);
    s_zA     = sin(zA * __DM_arcsec2r);
    c_zA     = cos(zA * __DM_arcsec2r);

    gsl_matrix_set(M_precession, 0, 0, -s_zA * s_zetaA + c_zA * c_thetaA * c_zetaA);
    gsl_matrix_set(M_precession, 0, 1, -s_zA * c_zetaA - c_zA * c_thetaA * s_zetaA);
    gsl_matrix_set(M_precession, 0, 2, -c_zA * s_thetaA);
    gsl_matrix_set(M_precession, 1, 0,  c_zA * s_zetaA + s_zA * c_thetaA * c_zetaA);
    gsl_matrix_set(M_precession, 1, 1,  c_zA * c_zetaA - s_zA * c_thetaA * s_zetaA);
    gsl_matrix_set(M_precession, 1, 2, -s_zA * s_thetaA);
    gsl_matrix_set(M_precession, 2, 0,  s_thetaA * c_zetaA);
    gsl_matrix_set(M_precession, 2, 1, -s_zetaA  * s_thetaA);
    gsl_matrix_set(M_precession, 2, 2,  c_thetaA);

    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, M_nutation, M_precession, 0.0, M_nut_n_pre);  //M_nut_n_pre = M_nutation * M_precession

    /*----------------------------------------------------------- */
    /*------------------- Rotation Matrix --------------------*/
    /*----------------------------------------------------------- */
    temps_sideral = UT1 +  (24110.54841 + 8640184.812866 * t + 0.093104 * t2 - 0.0000062 * t3);

    temps_sideral = temps_sideral * __DM_sec2r + delta_psi * cos(epsilonA) * __DM_arcsec2r; /* unit: radian */

    gsl_matrix_set(M_rotation, 0, 0, cos(temps_sideral));
    gsl_matrix_set(M_rotation, 0, 1, sin(temps_sideral));
    gsl_matrix_set(M_rotation, 0, 2, 0.0);
    gsl_matrix_set(M_rotation, 1, 0, -sin(temps_sideral));
    gsl_matrix_set(M_rotation, 1, 1, cos(temps_sideral));
    gsl_matrix_set(M_rotation, 1, 2, 0.0);
    gsl_matrix_set(M_rotation, 2, 0, 0.0);
    gsl_matrix_set(M_rotation, 2, 1, 0.0);
    gsl_matrix_set(M_rotation, 2, 2, 1.0);

    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, M_rotation, M_nut_n_pre, 0.0, TEIC);

    gsl_matrix_free(M_rotation);
    gsl_matrix_free(M_nutation);
    gsl_matrix_free(M_precession);
    gsl_matrix_free(M_nut_n_pre);
    return 0;
}

int build_WEII(gsl_matrix *WEII) {

  gsl_matrix_set(WEII, 0, 1, -__WEII3);
  gsl_matrix_set(WEII, 1, 0, __WEII3);

  return 0;
}

/*******************************************************************************
*   AccelHarmonic
*
*   Purpose:
*
*       Computes the acceleration of a launch vehicle due to
*           -The Earth's harmonic gravity field
*
*   Input:
*       r           Launch Vehicle position vector in ECI coordinate
*       E           Transformation matrix from ECI to ECEF coordinate
*       GM          Gravitational coefficient
*       R_ref       Earth mean Radius
*       CS          Spherical harmonic coefficients (un-normalized)
*       n_max       Maxium degree
*       m_max       Maxium orger (m_max<=n_max; m_max=0 for zonals, only)
*
*   Output:
*       acc         Gravitational acceleration
*
********************************************************************************/

int AccelHarmonic(const gsl_vector *SBII, double CS[21][21], int n_max, int m_max, const gsl_matrix *TEIC, gsl_vector *acc_out) {
    int    n, m;                /* Loop counters */
    double r_sqr, rho, Fac;     /* Auxiliary quantities */
    double x0, y0, z0;          /* Normalized coordinates */
    double ax, ay, az;          /* Acceleration vector */
    double C, S;                /* Gravitational coefficients */

    gsl_vector *r_bf, *a_bf;

    r_bf = gsl_vector_calloc(3);
    a_bf = gsl_vector_calloc(3);

    double V[N_JGM3+2][N_JGM3+2] = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    };  /* Harmonic functions */
    double W[N_JGM3+2][N_JGM3+2] = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    };  /* work array (0..n_max+1,0..n_max+1) */

    /* Earth-fixed position */
    gsl_blas_dgemv(CblasNoTrans, 1.0, TEIC, SBII, 0.0, r_bf);  // r_bf = TEIC * SBII

    gsl_blas_ddot(r_bf, r_bf, &r_sqr);  /* Square of distance */
    rho = __SMAJOR_AXIS * __SMAJOR_AXIS / r_sqr;

    x0 = __SMAJOR_AXIS * gsl_vector_get(r_bf, 0) / r_sqr;
    y0 = __SMAJOR_AXIS * gsl_vector_get(r_bf, 1) / r_sqr;
    z0 = __SMAJOR_AXIS * gsl_vector_get(r_bf, 2) / r_sqr;

    /*******************************
    *
    * Evaluate harmonic functions
    *   V_nm = (R_ref/r)^(n+1) * P_nm(sin(phi)) * cos(m*lambda)
    * and
    *   W_nm = (R_ref/r)^(n+1) * P_nm(sin(phi)) * sin(m*lambda)
    * up to degree and order n_max+1
    *
    ********************************/

    /* Calculate zonal terms V(n,0); set W(n,0)=0.0 */

    V[0][0] = __SMAJOR_AXIS / sqrt(r_sqr);
    W[0][0] = 0.0;

    V[1][0] = z0 * V[0][0];
    W[1][0] = 0.0;

    for (n = 2; n <= n_max+1; n++) {
        V[n][0] = ((2*n-1) * z0 * V[n-1][0] - (n-1) * rho * V[n-2][0]) / n;
        W[n][0] = 0.0;
    }

    /* Calculate tesseral and sectorial terms */
    for (m = 1; m <= m_max+1; m++) {
        /* Calculate V(m,m) .. V(n_max+1,m) */

        V[m][m] = (2*m-1) * (x0 * V[m-1][m-1] - y0 * W[m-1][m-1]);
        W[m][m] = (2*m-1) * (x0 * W[m-1][m-1] + y0 * V[m-1][m-1]);

        if (m <= n_max) {
            V[m+1][m] = (2*m+1) * z0 * V[m][m];
            W[m+1][m] = (2*m+1) * z0 * W[m][m];
        }

        for (n = m+2; n <= n_max+1; n++) {
            V[n][m] = ((2*n-1) * z0 * V[n-1][m] - (n+m-1) * rho * V[n-2][m]) / (n-m);
            W[n][m] = ((2*n-1) * z0 * W[n-1][m] - (n+m-1) * rho * W[n-2][m]) / (n-m);
        }
    }

    /* Calculate accelerations ax,ay,az */
    ax = ay = az = 0.0;

    for (m = 0; m <= m_max; m++) {
        for (n = m; n <= n_max ; n++) {
            if (m == 0) {
                C = CS[n][0];   /* = C_n,0 */
                ax -=       C * V[n+1][1];
                ay -=       C * W[n+1][1];
                az -= (n+1)*C * V[n+1][0];
            } else {
                C = CS[n][m];   /* = C_n,m */
                S = CS[m-1][n]; /* = S_n,m */
                Fac = 0.5 * (n-m+1) * (n-m+2);
                ax +=   + 0.5 * (- C * V[n+1][m+1] - S * W[n+1][m+1])
                        + Fac * (+ C * V[n+1][m-1] + S * W[n+1][m-1]);
                ay +=   + 0.5 * (- C * W[n+1][m+1] + S * V[n+1][m+1])
                        + Fac * (- C * W[n+1][m-1] + S * V[n+1][m-1]);
                az += (n-m+1) * (- C * V[n+1][m]   - S * W[n+1][m]);
            }
        }
    }

    /* Body-fixed acceleration */
    gsl_vector_set(a_bf, 0, (__GM / (__SMAJOR_AXIS * __SMAJOR_AXIS)) * ax);
    gsl_vector_set(a_bf, 1, (__GM / (__SMAJOR_AXIS * __SMAJOR_AXIS)) * ay);
    gsl_vector_set(a_bf, 2, (__GM / (__SMAJOR_AXIS * __SMAJOR_AXIS)) * az);

    /* Inertial acceleration */
    gsl_blas_dgemv(CblasTrans, 1.0, TEIC, a_bf, 0.0, acc_out);  // acc_out = trans(TEIC) * a_bf

    gsl_vector_free(r_bf);
    gsl_vector_free(a_bf);
    return 0;
}

int DCM_2_Euler_angle(const gsl_matrix *TBD, double *phibdc, double *thtbdc, double *psibdc) {
    double cthtbd;
    // double mroll = 0.0;

    double tbd13 = gsl_matrix_get(TBD, 0, 2);
    double tbd11 = gsl_matrix_get(TBD, 0, 0);
    double tbd33 = gsl_matrix_get(TBD, 2, 2);
    double tbd12 = gsl_matrix_get(TBD, 0, 1);
    double tbd23 = gsl_matrix_get(TBD, 1, 2);
    // *geodetic Euler angles
    // computed pitch angle: 'thtbdc'
    // note: when |tbd13| >= 1, thtbdc = +- pi/2, but cos(thtbdc) is
    // forced to be a small positive number to prevent division by zero
    if (fabs(tbd13) < 1) {
        *thtbdc = asin(-tbd13);
        cthtbd = cos(*thtbdc);
    } else {
        *thtbdc = ___PI / 2 * sign(-tbd13);
        cthtbd = __EPS;
    }

    // computed yaw angle: 'psibdc'
    double cpsi = tbd11 / cthtbd;
    if (fabs(cpsi) > 1) {
        cpsi = 1 * sign(cpsi);
    }
    *psibdc = acos(cpsi) * sign(tbd12);

    // computed roll angle: 'phibdc'
    double cphi = tbd33 / cthtbd;
    if (fabs(cphi) > 1) {
        cphi = 1 * sign(cphi);
    }
        
    *phibdc = acos(cphi) * sign(tbd23);

    return 0;
}

int calculate_INS_derived_phip(gsl_vector *VBECB, double *phipc) {
  // double VBECB0 = gsl_vector_get(VBECB, 0);
  double VBECB1 = gsl_vector_get(VBECB, 1);
  double VBECB2 = gsl_vector_get(VBECB, 2);

  if (VBECB1 == 0.0 && VBECB2 == 0.0) {
      *phipc = 0.0;
  } else if (fabs(VBECB1) < __EPS) {
      // note: if vbeb2 is <EPS the value if phipc is forced to be 0 or PI
      //       to prevent oscillations
      if (VBECB2 > 0.0) *phipc = 0.0;
      if (VBECB2 < 0.0) *phipc = ___PI;
  } else {
      *phipc = atan2(VBECB1, VBECB2);
  }

  return 0;
}

int calculate_INS_derived_thtvd(gsl_vector *VBECD, double *thtvd) {
    double VBECD0 = gsl_vector_get(VBECD, 0);
    double VBECD1 = gsl_vector_get(VBECD, 1);
    double VBECD2 = gsl_vector_get(VBECD, 2);

    if (VBECD0 == 0.0 && VBECD1 == 0.0) {
        *thtvd = 0.0;
    } else {
        *thtvd = atan2(-VBECD2, sqrt(VBECD0 * VBECD0 + VBECD1 * VBECD1));
    }

    return 0;
}

int calculate_INS_derived_psivd(gsl_vector *VBECD, double *psivd) {
    double VBECD0 = gsl_vector_get(VBECD, 0);
    double VBECD1 = gsl_vector_get(VBECD, 1);

    if (VBECD0 == 0.0 && VBECD1 == 0.0) {
        *psivd = 0.0;
    } else {
        *psivd = atan2(VBECD1, VBECD0);
    }

    return 0;
}

int calculate_INS_derived_alpp(gsl_vector *VBECB, double *alpp) {
    double dum = gsl_vector_get(VBECB, 0) / gsl_blas_dnrm2(VBECB);
    if (fabs(dum) > 1.0) {
        dum = 1.0 * sign(dum);
    }
    *alpp = acos(dum);

    return 0;
}

int calculate_INS_derived_beta(gsl_vector *VBECB, double *beta) {
    *beta = asin(gsl_vector_get(VBECB, 1) / gsl_blas_dnrm2(VBECB));
    return 0;
}

int calculate_INS_derived_alpha(gsl_vector *VBECB, double *alpha) {
    *alpha = atan2(gsl_vector_get(VBECB, 2), gsl_vector_get(VBECB, 0));
    return 0;
}

int build_VBEB(double _alpha0x, double _beta0x, double _dvbe, gsl_vector *VBEB) {
    double salp = sin(_alpha0x * __RAD);
    double calp = cos(_alpha0x * __RAD);
    double sbet = sin(_beta0x * __RAD);
    double cbet = cos(_beta0x * __RAD);
    // double vbeb3 = salp * cbet * _dvbe;

    gsl_vector_set(VBEB, 0, calp * cbet * _dvbe);
    gsl_vector_set(VBEB, 1, sbet * _dvbe);
    gsl_vector_set(VBEB, 2, salp * cbet * _dvbe);

    return 0;
}

int INS_update(const double int_step, double *dvbec, unsigned int liftoff, double *alphacx, double *betacx
                , double *alppcx, double *phipcx, double *loncx, double *latcx, double *altc, double *psivdcx, double *thtvdcx
                , double *phibdc, double *thtbdc, double *psibdc
                , gsl_vector *WBICB, gsl_vector *PHI, gsl_vector *DELTA_VEL
                , gsl_vector *PHI_HIGH, gsl_vector *PHI_LOW, GPS_TIME gps, gsl_matrix *TEIC
                , gsl_vector *SBIIC, gsl_vector *GRAVGI, gsl_matrix *TBIC, gsl_vector *SBEEC
                , gsl_vector *VBEEC, gsl_matrix *WEII, gsl_matrix *TLI, gsl_matrix *TDCI
                , gsl_matrix *TBICI, gsl_matrix *TBDC, gsl_vector *TBDCQ) {
    gsl_vector *VBIIC_old, *VBIIC_tmp1, *VBIIC_tmp2, *VBEEC_tmp1, *VBEEC_tmp2, *VBECB, *VBECB_tmp, *VBECD, *VBECD_tmp;
    gsl_matrix *TBIC_tmp1, *TBIC_tmp2;
    VBIIC_old = gsl_vector_calloc(3);
    VBIIC_tmp1 = gsl_vector_calloc(3);
    VBIIC_tmp2 = gsl_vector_calloc(3);
    VBEEC_tmp1 = gsl_vector_calloc(3);
    VBEEC_tmp2 = gsl_vector_calloc(3);
    VBECB = gsl_vector_calloc(3);
    VBECB_tmp = gsl_vector_calloc(3);
    VBECD = gsl_vector_calloc(3);
    VBECD_tmp = gsl_vector_calloc(3);
    TBIC_tmp1 = gsl_matrix_calloc(3, 3);
    TBIC_tmp2 = gsl_matrix_calloc(3, 3);

    calculate_INS_derived_TEI(gps, TEIC);
    AccelHarmonic(SBIIC, INS_CS_JGM3, 20, 20, TEIC, GRAVGI);
    gsl_vector_memcpy(VBIIC_tmp2, GRAVGI);

    gsl_vector_memcpy(VBIIC_old, VBIIC);  // VBIIC_old = VBIIC

    /* VBIIC += trans(TBIC) * DELTA_VEL + GRAVGI * int_step */
    gsl_blas_dgemv(CblasTrans, 1.0, TBIC, DELTA_VEL, 0.0, VBIIC_tmp1);
    gsl_blas_dscal(int_step, VBIIC_tmp2);
    gsl_vector_add(VBIIC, VBIIC_tmp1);
    gsl_vector_add(VBIIC, VBIIC_tmp2);

    /* SBIIC += VBIIC_old * int_step */
    gsl_blas_dscal(int_step, VBIIC_old);
    gsl_vector_add(SBIIC, VBIIC_old);

    /* TBIC = build_321_rotation_matrix(PHI) * TBIC */
    build_321_rotation_matrix(PHI, TBIC_tmp1);
    gsl_matrix_memcpy(TBIC_tmp2, TBIC);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, TBIC_tmp1, TBIC_tmp2, 0.0, TBIC);

    /* SBEEC = TEIC * SBIIC */
    gsl_blas_dgemv(CblasNoTrans, 1.0, TEIC, SBIIC, 0.0, SBEEC);

    /* VBEEC = TEIC * VBIIC - WEII * SBEEC */
    gsl_blas_dgemv(CblasNoTrans, 1.0, TEIC, VBIIC, 0.0, VBEEC_tmp1);
    gsl_blas_dgemv(CblasNoTrans, 1.0, WEII, SBEEC, 0.0, VBEEC_tmp2);
    for (int i = 0; i < 3; i++) gsl_vector_set(VBEEC, i, gsl_vector_get(VBEEC_tmp1, i) - gsl_vector_get(VBEEC_tmp2, i));

    gsl_blas_dgemv(CblasTrans, 1.0, TEIC, VBEEC, 0.0, VBECB_tmp);
    gsl_blas_dgemv(CblasNoTrans, 1.0, TBIC, VBECB_tmp, 0.0, VBECB);

    *dvbec = gsl_blas_dnrm2(VBECB);

    if (liftoff == 1) {
        // computing indidence angles from INS
        calculate_INS_derived_alpha(VBECB, alphacx);
        *alphacx *= __DEG;
        calculate_INS_derived_beta(VBECB, betacx);
        *betacx *= __DEG;
        calculate_INS_derived_alpp(VBECB, alppcx);
        *alppcx *= __DEG;
        calculate_INS_derived_phip(VBECB, phipcx);
        *phipcx *= __DEG;
    }

    cad_geo84_in(SBIIC, TEIC, loncx, latcx, altc);
    *loncx *= __DEG;
    *latcx *= __DEG;

    // getting T.M. of geodetic wrt inertial coord
    cad_tdi84(*loncx * __RAD, *latcx * __RAD, *altc, TEIC, TDCI);

    // getting Launch site T.M. of inertial coord
    if (liftoff == 0) gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, TLI, TDCI, 0.0, TBICI);  //  TBICI = TLI * TDCI

    // computing geodetic velocity from INS
    gsl_blas_dgemv(CblasNoTrans, 1.0, TEIC, VBEEC, 0.0, VBECD_tmp);
    gsl_blas_dgemv(CblasTrans, 1.0, TDCI, VBECD_tmp, 0.0, VBECD);

    // computing flight path angles
    calculate_INS_derived_psivd(VBECD, psivdcx);
    *psivdcx *= __DEG;
    calculate_INS_derived_thtvd(VBECD, thtvdcx);
    *thtvdcx *= __DEG;

    // computing Euler angles from INS
    gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, TBIC, TDCI, 0.0, TBDC);
    Matrix2Quaternion_C(TBDC, TBDCQ);
    DCM_2_Euler_angle(TBDC, phibdc, thtbdc, psibdc);

    gsl_vector_free(VBIIC_old);
    gsl_vector_free(VBIIC_tmp1);
    gsl_vector_free(VBIIC_tmp2);
    gsl_vector_free(VBEEC_tmp1);
    gsl_vector_free(VBEEC_tmp2);
    gsl_vector_free(VBECB);
    gsl_vector_free(VBECB_tmp);
    gsl_vector_free(VBECD);
    gsl_vector_free(VBECD_tmp);
    gsl_matrix_free(TBIC_tmp1);
    gsl_matrix_free(TBIC_tmp2);

    return 0;
}


int load_location(double lonx, double latx, double alt) {
    loncx = lonx;
    latcx = latx;
    altc = alt;

    return 0;
}
int load_angle(double yaw, double roll, double pitch, GPS_TIME gps_time) {
    phibdcx = roll;
    thtbdcx = pitch;
    psibdcx = yaw;

    calculate_INS_derived_TEI(gps_time, TEIC);

    build_psi_tht_phi_TM_C(psibdcx * __RAD, thtbdcx * __RAD, phibdcx * __RAD, TBD);
    build_psi_tht_phi_TM_C(psibdcx * __RAD, thtbdcx * __RAD, phibdcx * __RAD, TLI);

    cad_tdi84(loncx * __RAD, latcx * __RAD, altc, TEIC, TDCI);

    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, TBD, TDCI, 0.0, TBIC);
    Matrix2Quaternion_C(TBIC, TBIC_Q);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, TLI, TDCI, 0.0, TBICI);

    return 0;
}
int load_geodetic_velocity(double alpha0x, double beta0x, double dvbe) {
    gsl_vector *VBEB, *VBIIC_tmp1, *VBIIC_tmp2, *VBIIC_tmp3, *VBIIC_tmp4;
    VBEB = gsl_vector_calloc(3);
    VBIIC_tmp1 = gsl_vector_calloc(3);
    VBIIC_tmp2 = gsl_vector_calloc(3);
    VBIIC_tmp3 = gsl_vector_calloc(3);
    VBIIC_tmp4 = gsl_vector_calloc(3);

    build_VBEB(alpha0x, beta0x, dvbe, VBEB);
    gsl_blas_dgemv(CblasTrans, 1.0, TBD, VBEB, 0.0, VBECD);
    cad_in_geo84(loncx * __RAD, latcx * __RAD, altc, TEIC, SBIIC);

    /** VBIIC calculate **/
    VBIIC = gsl_vector_calloc(3);
    gsl_blas_dgemv(CblasNoTrans, 1.0, TEIC, SBIIC, 0.0, VBIIC_tmp4);
    gsl_blas_dgemv(CblasNoTrans, 1.0, WEII, VBIIC_tmp4, 0.0, VBIIC_tmp3);
    gsl_blas_dgemv(CblasTrans, 1.0, TEIC, VBIIC_tmp3, 0.0, VBIIC_tmp2);
    gsl_blas_dgemv(CblasTrans, 1.0, TDCI, VBECD, 0.0, VBIIC_tmp1);
    gsl_vector_add(VBIIC, VBIIC_tmp1);
    gsl_vector_add(VBIIC, VBIIC_tmp2);

    return 0;
}

int INS_init(GPS_TIME gps_time) {
    build_WEII(WEII);
    calculate_INS_derived_TEI(gps_time, TEIC);
    cad_in_geo84(loncx * __RAD, latcx * __RAD, altc, TEIC, SBIIC);
    AccelHarmonic(SBIIC, INS_CS_JGM3, 20, 20, TEIC, GRAVGI);
    gsl_blas_dgemv(CblasNoTrans, 1.0, TEIC, SBIIC, 0.0, SBEEC);
    gsl_vector *VBEEC_tmp1;
    VBEEC_tmp1 = gsl_vector_calloc(3);
    gsl_vector_set_zero(VBEEC);

    gsl_blas_dgemv(CblasNoTrans, 1.0, TEIC, VBIIC, 0.0, VBEEC);
    gsl_blas_dgemv(CblasNoTrans, 1.0, WEII, SBEEC, 0.0, VBEEC_tmp1);

    gsl_vector_sub(VBEEC, VBEEC_tmp1);

    return 0;
}

int INS_alloc() {
    /** Matrix **/
    WEII = gsl_matrix_calloc(3, 3);
    TBIC = gsl_matrix_calloc(3, 3);
    TDCI = gsl_matrix_calloc(3, 3);
    TEIC = gsl_matrix_calloc(3, 3);
    TBD = gsl_matrix_calloc(3, 3);
    TBICI = gsl_matrix_calloc(3, 3);
    TLI = gsl_matrix_calloc(3, 3);
    /** Vector **/
    EVBI = gsl_vector_calloc(3);
    EVBID = gsl_vector_calloc(3);
    ESBI = gsl_vector_calloc(3);
    ESBID = gsl_vector_calloc(3);
    RICI = gsl_vector_calloc(3);
    RICID = gsl_vector_calloc(3);
    TBIC_Q = gsl_vector_calloc(4);
    TBIDC_Q = gsl_vector_calloc(4);
    SBIIC = gsl_vector_calloc(3);
    VBIIC = gsl_vector_calloc(3);
    SBEEC = gsl_vector_calloc(3);
    VBEEC = gsl_vector_calloc(3);
    WBICI = gsl_vector_calloc(3);
    EGRAVI = gsl_vector_calloc(3);
    VBECD = gsl_vector_calloc(3);
    INS_I_ATT_ERR = gsl_vector_calloc(3);
    TESTV = gsl_vector_calloc(3);
    TMP_old = gsl_vector_calloc(3);
    VBIIC_old = gsl_vector_calloc(3);
    POS_ERR = gsl_vector_calloc(3);
    GRAVGI = gsl_vector_calloc(3);
    TBDQ = gsl_vector_calloc(4);
    VBIIC_old_old = gsl_vector_calloc(3);

    return 0;
}














