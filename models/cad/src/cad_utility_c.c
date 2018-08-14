#include "cad_utility_c.h"
/*
 * @brief Returns the inertial displacement vector from longitude, latitude and
 *      altitude
 *      using the WGS 84 reference ellipsoid
 *      Reference: 
 *      1. Britting,K.R."Inertial Navigation Systems Analysis" pp.45-49, Wiley, 1971
 *      2. Geodetic_Coordinate_Conversion.pdf James R. Clynch February 2006 p.3
 *
 * @param[in] lon = geodetic longitude - rad
 * @param[in] lat = geodetic latitude - rad
 * @param[in] alt = altitude above ellipsoid - m
 * @param[in] TEI = T.M from Inertia coordinate to ECEF coordinate
 * @param[out] SBII(3x1) = Position vector in Inertia coordinate - m
 *
 * @author 030411 Created from FORTRAN by Peter H Zipfel
 * @author 170121 Create Armadillo Version by soncyanga
 * @author 20180719 Create GSL Version by CHUN-HSU LAI
 */
int cad_in_geo84(const double lon, const double lat
                    , const double alt, const gsl_matrix *TEI, gsl_vector *SBII) {
    gsl_vector *SBIE = gsl_vector_calloc(3);
    double r0, e, dbi;

    r0 = __SMAJOR_AXIS / sqrt(1.0 - __FLATTENING * (2.0 - __FLATTENING) * sin(lat) * sin(lat));
    e = sqrt(2.0 * __FLATTENING - __FLATTENING * __FLATTENING);
    dbi = r0 + alt;
    gsl_vector_set(SBIE, 0, dbi * cos(lat) * cos(lon));
    gsl_vector_set(SBIE, 1, dbi * cos(lat) * sin(lon));
    gsl_vector_set(SBIE, 2, ((1.0 - e * e) * r0 + alt) * sin(lat));
    gsl_blas_dgemv(CblasTrans, 1.0, TEI, SBIE, 0.0, SBII);  //  SBII = Trans(TEI) * SBIE

    gsl_vector_free(SBIE);
    return 0;
}

/*
 * @brief Returns the T.M. of geodetic wrt inertial coordinates
 *        using the WGS 84 reference ellipsoid
 *
 * @return TDI(3x3) = T.M.of geosetic wrt inertial coord - ND
 *
 * @param[in] lon = geodetic longitude - rad
 * @param[in] lat = geodetic latitude - rad
 * @param[in] alt = altitude above ellipsoid - m
 * @param[in] TEI = T.M from Inertia coordinate to ECEF coordinate
 *
 * @author 030424 Created by Peter H Zipfel
 * @author 170121 Create Armadillo Version by sonicyang
 * @author 20180719 Create GSL version by CHUN-HSU LAI
 */

int cad_tdi84(const double lon, const double lat, const double alt, const gsl_matrix *TEI, gsl_matrix *TDI) {
    gsl_matrix *TDE = gsl_matrix_calloc(3, 3);
    double tde13, tde33, tde22, tde21;
    tde13 = cos(lat);
    tde33 = -sin(lat);
    tde22 = cos(lon);
    tde21 = -sin(lon);

    gsl_matrix_set(TDE, 0, 0, tde33 * tde22);
    gsl_matrix_set(TDE, 0, 1, -tde33 * tde21);
    gsl_matrix_set(TDE, 0, 2, tde13);
    gsl_matrix_set(TDE, 1, 0, tde21);
    gsl_matrix_set(TDE, 1, 1, tde22);
    gsl_matrix_set(TDE, 1, 2, 0.0);
    gsl_matrix_set(TDE, 2, 0, -tde13 * tde22);
    gsl_matrix_set(TDE, 2, 1, tde13 * tde21);
    gsl_matrix_set(TDE, 2, 2, tde33);

    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, TDE, TEI, 0.0, TDI); //  TDI = TDE * TEI
    gsl_matrix_free(TDE);
    return 0;
}

/**
 * @brief Calculates geodetic longitude, latitude, and altitude from inertial
 *        displacement vector
 *        using the WGS 84 reference ellipsoid
 *        Reference: Britting,K.R."Inertial Navigation Systems Analysis", Wiley. 1971
 *
 * @param[in] SBII(3x1) = Inertial position - m
 * @param[in] TEI(3x3) = T.M from Inertia coordinate to ECEF coordinate
 * @param[out] lon = Lontitude - deg
 * @param[out] lat = Latitude - deg
 * @param[out] alt = Altitude - m
 *
 * @author 030414 Created from FORTRAN by Peter H Zipfel
 * @author 170121 Create Armadillo Version by soncyang
 * @author 20180719 Create GSL version by CHUN-HSU LAI
 */
int cad_geo84_in(const gsl_vector *SBII, const gsl_matrix *TEI, double *lon, double *lat, double *alt) {
    int count = 0;
    double lat0;
    double alamda;
    double dbi, latg, r0, dd, sbee1, sbee2, dum4;
    gsl_vector *SBEE = gsl_vector_calloc(3);

    gsl_blas_dgemv(CblasNoTrans, 1.0, TEI, SBII, 0.0, SBEE);  //  SBEE = TEI * SBII

    dbi = gsl_blas_dnrm2(SBEE);
    latg = asin(gsl_vector_get(SBEE, 2) / dbi);

    *lat = latg;

    /** iterating to calculate geodetic latitude and altitude */
    do {
        lat0 = *lat;
        r0 = __SMAJOR_AXIS * (1.0 - __FLATTENING * (1.0 - cos(2.0 * lat0)) / 2.0 +
             5.0 * __FLATTENING * __FLATTENING * (1.0 - cos(4.0 * lat0)) / 16.0);  /** eq 4-21 */
        *alt = dbi - r0;
        dd = __FLATTENING * sin(2.0 * lat0) * (1.0 - __FLATTENING / 2.0 - *alt / r0); /** eq 4-15 */
        *lat = latg + dd;
        count++;
        assert(count <= 100 &&
               " *** Stop: Geodetic latitude does not "
               "converge,'cad_geo84_in()' *** ");
    }while (fabs(*lat - lat0) > __SMALL);

    /** Longtitude **/

    sbee1 = gsl_vector_get(SBEE, 0);
    sbee2 = gsl_vector_get(SBEE, 1);
    dum4 = asin(sbee2 / sqrt(sbee1 * sbee1 + sbee2 * sbee2));

    /** Resolving the multi-valued arcsin function */
    if ((sbee1 >= 0.0) && (sbee2 >= 0.0))
        alamda = dum4;  /** quadrant I */
    if ((sbee1 < 0.0) && (sbee2 >= 0.0))
        alamda = (180. * __RAD) - dum4;  /** quadrant II */
    if ((sbee1 < 0.0) && (sbee2 < 0.0))
        alamda = (180. * __RAD) - dum4;  /** quadrant III */
    if ((sbee1 > 0.0) && (sbee2 < 0.0))
        alamda = (360. * __RAD) + dum4;  /** quadrant IV */
    *lon = alamda;  /** - WEII3 * time - GW_CLONG; */
    if ((*lon) > (180. * __RAD))
        *lon = -((360. * __RAD) - *lon);  /** east positive, west negative */

    gsl_vector_free(SBEE);
    return 0;
}

/**
 * @brief Projects initial state through 'tgo' to final state along a Keplerian
 *      trajectory
 *      Based on Ray Morth, unpublished utility
 *
 * @return kepler_flan = 0: good Kepler projection;
 *                     = 1: bad (# of iterations>20, or neg. sqrt), no new proj cal,
 *                          use prev value;  - ND
 * @param[out] SPII = projected inertial position after tgo - m
 * @param[out] VPII = projected inertial velocity after tgo - m/s
 *
 * @param[in] SBII = current inertial position - m
 * @param[in] VBII = current inertial velocity - m/s
 * @param[in] tgo = time-to-go to projected point - sec
 *
 * @author 040319 Created from FORTRAN by Peter H Zipfel
 * @author 201807020 Create GSL version by CHUN-HSU LAI
 */

int cad_kepler(const gsl_vector *SBII, const gsl_vector *VBII, const double tgo
                , gsl_vector *SPII, gsl_vector *VPII) {
    gsl_vector_set_zero(SPII);  //  set SPII to zero
    gsl_vector_set_zero(VPII);  //  set VPII to zero

    double sde, cde, sqrt_GM, ro, vo, rvo
            , a1, sa, smua, mdot, dm, de, a11
            , a21, adm, dmn, dmerr, dmde, fk, gk
            , rp, fdk, gdk;
    int kepler_flag, count20;
    kepler_flag = 0;
    count20 = 0;

    sqrt_GM = sqrt(__GM);
    ro = gsl_blas_dnrm2(SBII);
    vo = gsl_blas_dnrm2(VBII);
    gsl_blas_ddot(SBII, VBII, &rvo);
    a1 = vo * vo / __GM;
    sa = ro / (2.0 - ro * a1);

    if (sa < 0.0) {
        // return without re-calculating SPII, VPII
        kepler_flag = 1;
        return kepler_flag;
    }

    smua = sqrt_GM * sqrt(sa);
    mdot = smua / (sa * sa);

    // calculating 'de'iteratively
    dm = mdot * tgo;
    de = dm;  //  initialize eccentricity
    a11 = rvo / smua;
    a21 = (sa - ro) / sa;

    do {
        cde = 1.0 - cos(de);
        sde = sin(de);
        dmn = de + a11 * cde - a21 * sde;
        dmerr = dm - dmn;

        adm = fabs(dmerr) / mdot;
        dmde = 1.0 + a11 * sde - a21 * (1.0 - cde);
        de = de + dmerr / dmde;
        count20++;
        if (count20 > 20) {
            // return without re-calculating SPII, VPII
            kepler_flag = 1;
            return kepler_flag;
        }
    }while (adm > __SMALL);

    fk = (ro - sa * cde) / ro;
    gk = (dm + sde - de) / mdot;
    
    /** Calculate SPII **/
    gsl_vector *SPII_temp1 = gsl_vector_calloc(3);
    gsl_vector *SPII_temp2 = gsl_vector_calloc(3);
    
    gsl_vector_memcpy(SPII_temp1, SBII);
    gsl_vector_memcpy(SPII_temp2, VBII);

    gsl_blas_dscal(fk, SPII_temp1);
    gsl_blas_dscal(gk, SPII_temp2);

    gsl_vector_add(SPII, SPII_temp1);
    gsl_vector_add(SPII, SPII_temp2);

    rp = gsl_blas_dnrm2(SPII);
    fdk = -smua * sde / ro;
    gdk = rp - sa * cde;

    /** Calculate VPII **/
    gsl_vector *VPII_temp1 = gsl_vector_calloc(3);
    gsl_vector *VPII_temp2 = gsl_vector_calloc(3);
    
    gsl_vector_memcpy(VPII_temp1, SBII);
    gsl_vector_memcpy(VPII_temp2, VBII);

    gsl_blas_dscal(fdk / rp, VPII_temp1);
    gsl_blas_dscal(gdk / rp, VPII_temp2);

    gsl_vector_add(VPII, VPII_temp1);
    gsl_vector_add(VPII, VPII_temp2);

    /** Dealloc memory **/
    gsl_vector_free(SPII_temp1);
    gsl_vector_free(SPII_temp2);
    gsl_vector_free(VPII_temp1);
    gsl_vector_free(VPII_temp2);

    return kepler_flag;
}









