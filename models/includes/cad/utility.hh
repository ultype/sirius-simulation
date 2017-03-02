#ifndef __CAD_UTIL_HH__
#define __CAD_UTIL_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (CAD Utility functions from Rocket6G)
LIBRARY DEPENDENCY:
      ((../../src/cad/utility.cpp))
*******************************************************************************/
#include <armadillo>

namespace cad {
    ///////////////////////////////////////////////////////////////////////////////
    // Returns great circle distance between two point on a spherical Earth
    // Reference: Bate et al. "Fundamentals of Astrodynamics", Dover 1971, p. 310
    // Return output
    //    distancex = great circle distance - km
    // Parameter input
    //    lon1 = longitude of first point - rad
    //    lat1 = latitude -of first point  rad
    //    lon2 = longitude of second point - rad
    //    lat2 = latitude -of second point  rad
    //
    // 030414 Created from FORTRAN by Peter H Zipfel
    ///////////////////////////////////////////////////////////////////////////////
    double distance(const double &lon1,
                    const double &lat1,
                    const double &lon2,
                    const double &lat2);

    //////////////////////////////////////////////////////////////////////////////
    // Calculates geodetic longitude, latitude, and altitude from inertial
    // displacement vector
    // using the WGS 84 reference ellipsoid
    // Reference: Britting,K.R."Inertial Navigation Systems Analysis", Wiley. 1971
    //
    // Parameter output
    //    lon = geodetic longitude - rad
    //    lat = geodetic latitude - rad
    //    alt = altitude above ellipsoid - m
    // Parameter input
    //    lat = geodetic latitude - rad
    //    SBII(3x1) = Inertial position - m
    //
    // 030414 Created from FORTRAN by Peter H Zipfel
    // 170121 Create Armadillo Version by soncyang
    ///////////////////////////////////////////////////////////////////////////////
    void geo84_in(double &lon,
                  double &lat,
                  double &alt,
                  arma::vec3 SBII,
                  const double &time);

    //////////////////////////////////////////////////////////////////////////////
    // Returns geodetic velocity vector information from inertial postion and
    // velocity
    // using the WGS 84 reference ellipsoid
    //
    // Calls utilities
    //    geo84_in(...), tdi84(...)
    // Parameter output
    //    dvbe = geodetic velocity - m/s
    //    psivdx = geodetic heading angle - deg
    //    thtvdx = geodetic flight path angle - deg
    // Parameter input
    //    SBII(3x1) = Inertial position - m
    //    VBII(3x1) = Inertial velocity - m
    //
    // 040710 created by Peter H Zipfel
    ///////////////////////////////////////////////////////////////////////////////
    void geo84vel_in(double &dvbe,
                     double &psivdx,
                     double &thtvdx,
                     arma::vec3 SBII,
                     arma::vec3 VBII,
                     const double &time);

    ////////////////////////////////////////////////////////////////////////////////
    // Returns geocentric lon, lat, alt from inertial displacement vector
    // for spherical Earth
    //
    // Parameter output
    //    lonc = geocentric longitude - rad
    //    latc = geocentric latitude - rad
    //    altc = geocentric latitude - m
    // Parameter input
    //    SBII = Inertial position - m
    //    time = simulation time - sec
    //
    // 010628 Created by Peter H Zipfel
    // 030416 Modified for SBII (not SBIE) input, PZi
    ////////////////////////////////////////////////////////////////////////////////
    void geoc_in(double &lonc,
                 double &latc,
                 double &altc,
                 arma::vec3 SBII,
                 const double &time);

    ////////////////////////////////////////////////////////////////////////////////
    // Returns lon, lat, alt from displacement vector in Earth coord for spherical
    // earth
    //
    // Return output
    //    RETURN[0]=lon
    //    RETURN[1]=lat
    //    RETURN[2]=alt
    // Paramter input
    //    SBIE = displacement of vehicle wrt Earth center in Earth coordinates
    //
    // 010628 Created by Peter H Zipfel
    ////////////////////////////////////////////////////////////////////////////////
    arma::vec3 geoc_ine(arma::vec3 SBIE);

    ///////////////////////////////////////////////////////////////////////////////
    // Earth gravitational acceleration, using the WGS 84 ellipsoid
    // Ref: Chatfield, A.B.,"Fundamentals of High Accuracy Inertial
    // Navigation",p.10, Prog.Astro and Aeronautics, Vol 174, AIAA, 1997.
    //
    // Return output
    //    GRAVG(3x1) = gravitational acceleration in geocentric coord - m/s^2
    // Parameter input
    //    SBII = inertial displacement vector - m
    //    time = simulation time - sec
    //
    // 030417 Created from FORTRAN by Peter H Zipfel
    ///////////////////////////////////////////////////////////////////////////////
    arma::vec3 grav84(arma::vec3 SBII, const double &time);

    ///////////////////////////////////////////////////////////////////////////////
    // Returns the inertial displacement vector from longitude, latitude and
    // altitude
    // using the WGS 84 reference ellipsoid
    // Reference: Britting,K.R."Inertial Navigation Systems Analysis"
    // pp.45-49, Wiley, 1971
    //
    // Return output
    //     SBII(3x1) = Inertial vehicle position - m
    // Parameter input
    //     lon = geodetic longitude - rad
    //     lat = geodetic latitude - rad
    //     alt = altitude above ellipsoid - m
    //     time = simulation time - sec
    //
    // 030411 Created from FORTRAN by Peter H Zipfel
    // 170121 Create Armadillo Version by soncyang
    ///////////////////////////////////////////////////////////////////////////////
    arma::vec3 in_geo84(const double lon,
                        const double lat,
                        const double alt,
                        const double &time);

    ///////////////////////////////////////////////////////////////////////////////
    // Returns the inertial displacement vector from geocentric longitude, latitude
    // and altitude
    // for spherical Earth
    //
    // Return output
    //    SBII = position of vehicle wrt center of Earth, in inertial coord
    // Argument input
    //    lon = geographic longitude - rad
    //    lat = geocentric latitude - rad
    //    alt = altitude above spherical Earth = m
    //
    // 010405 Created by Peter H Zipfel
    ///////////////////////////////////////////////////////////////////////////////
    arma::vec3 in_geoc(const double &lon,
                       const double &lat,
                       const double &alt,
                       const double &time);

    ///////////////////////////////////////////////////////////////////////////////
    // Calculates inertial displacement and velocity vectors from orbital elements
    // Reference: Bate et al. "Fundamentals of Astrodynamics", Dover 1971, p.71
    //
    // Return output
    //    parabola_flag = 0 ok
    //                  = 1 not suitable (divide by zero), because parabolic
    //                    trajectory
    // Parameter output
    //    SBII = Inertial position - m
    //    SBII = Inertial velocity - m/s
    // Parameter input
    //    semi = semi-major axis of orbital ellipsoid - m
    //    ecc = eccentricity of elliptical orbit - ND
    //    inclx = inclination of orbital wrt equatorial plane - deg
    //    lon_anodex = celestial longitude of the ascending node - deg
    //    arg_perix = argument of periapsis (ascending node to periapsis) - deg
    //    true_anomx = true anomaly (periapsis to satellite) - deg
    //
    // 040510 Created by Peter H Zipfel
    ///////////////////////////////////////////////////////////////////////////////
    int in_orb(arma::vec3 &SBII,
               arma::vec3 &VBII,
               const double &semi,
               const double &ecc,
               const double &inclx,
               const double &lon_anodex,
               const double &arg_perix,
               const double &true_anomx);

    ////////////////////////////////////////////////////////////////////////////////
    // Projects initial state through 'tgo' to final state along a Keplerian
    // trajectory
    // Based on Ray Morth, unpublished utility
    //
    // Return output
    //    kepler_flag = 0: good Kepler projection;
    //                = 1: bad (# of iterations>20, or neg. sqrt), no new proj cal,
    //                  use prev value;  - ND
    // Parameter output
    //    SPII = projected inertial position after tgo - m
    //    VPII = projected inertial velocity after tgo - m/s
    // Parameter input
    //    SBII = current inertial position - m
    //    VBII = current inertial velocity - m/s
    //    tgo = time-to-go to projected point - sec
    //
    // 040319 Created from FORTRAN by Peter H Zipfel
    ///////////////////////////////////////////////////////////////////////////////
    int kepler(arma::vec3 &SPII,
               arma::vec3 &VPII,
               arma::vec3 SBII,
               arma::vec3 VBII,
               const double &tgo);

    ////////////////////////////////////////////////////////////////////////////////
    // Projects initial state through 'tgo' to final state along a Keplerian
    // trajectory
    // Based on: Bate, Mueller, White, "Fundamentals of Astrodynamics", Dover 1971
    //
    // Return output
    //    iter_flag=0: # of iterations < 20; =1: # of iterations > 20;  - ND
    // Parameter output
    //    SPII = projected inertial position after tgo - m
    //    VPII = projected inertial velocity after tgo - m/s
    // Parameter input
    //    SBII = current inertial position - m
    //    VBII = current inertial velocity - m/s
    //    tgo = time-to-go to projected point - sec
    //
    // 040318 Created from ASTRO_KEP by Peter H Zipfel
    ///////////////////////////////////////////////////////////////////////////////
    int kepler1(arma::vec3 &SPII,
                arma::vec3 &VPII,
                arma::vec3 SBII,
                arma::vec3 VBII,
                const double &tgo);

    ////////////////////////////////////////////////////////////////////////////////
    // Calculates utility functions c(z) and s(z) for kepler(...)
    // Reference: Bate, Mueller, White, "Fundamentals of Astrodynamics", Dover 1971,
    //            p.196
    //
    // Paramter output
    //    c = c(z) utility function
    //    s = s(z) utility function
    // Parameter input
    //    z = z-variable
    //
    // 040318 Created from ASTRO_UCS by Peter H Zipfel
    ///////////////////////////////////////////////////////////////////////////////
    void kepler1_ucs(double &c, double &s, const double &z);

    ///////////////////////////////////////////////////////////////////////////////
    // Calculates the orbital elements from inertial displacement and velocity
    // Reference: Bate et al. "Fundamentals of Astrodynamics", Dover 1971, p.58
    //
    // Return output
    //    cadorbin_flag = 0 ok
    //                    1 'true_anomx' not calculated, because of circular orbit
    //                    2 'semi' not calculated, because parabolic orbit
    //                    3 'lon_anodex' not calculated, because equatorial orbit
    //                   13 'arg_perix' not calculated, because equatorialand/or
    //                      circular orbit
    // Parameter output:
    //    semi = semi-major axis of orbital ellipsoid - m
    //    ecc = eccentricity of elliptical orbit - ND
    //    inclx = inclination of orbital wrt equatorial plane - deg
    //    lon_anodex = celestial longitude of the ascending node - deg
    //    arg_perix = argument of periapsis (ascending node to periapsis) - deg
    //    true_anomx = true anomaly (periapsis to satellite) - deg
    //
    // Parameter input:
    //    SBII = Inertial position - m
    //    VBII = Inertial velocity - m/s
    //
    // 040510 Created by Peter H Zipfel
    // 170121 Create Armadillo Version by soncyang
    ///////////////////////////////////////////////////////////////////////////////
    int orb_in(double &semi,
                       double &ecc,
                       double &inclx,
                       double &lon_anodex,
                       double &arg_perix,
                       double &true_anomx,
                       arma::vec3 &SBII,
                       arma::vec3 &VBII);

    ///////////////////////////////////////////////////////////////////////////////
    // Returns the T.M. of geodetic wrt inertial coordinates
    // using the WGS 84 reference ellipsoid
    //
    // Return output
    //    TDI(3x3) = T.M.of geosetic wrt inertial coord - ND
    // Parameter input
    //    lon = geodetic longitude - rad
    //    lat = geodetic latitude - rad
    //    alt = altitude above ellipsoid - m
    //
    // 030424 Created by Peter H Zipfel
    // 170121 Create Armadillo Version by soncyang
    ///////////////////////////////////////////////////////////////////////////////
    arma::mat33 tdi84(const double &lon,
                               const double &lat,
                               const double &alt,
                               const double &time);

    ////////////////////////////////////////////////////////////////////////////////
    // Returns the T.M. of earth wrt inertial coordinates
    //
    // Return output
    //    TEI = T.M. of Earthy wrt inertial coordinates
    //
    // Argument input
    //    time = time since start of simulation - s
    //
    // 010628 Created by Peter H Zipfel
    ////////////////////////////////////////////////////////////////////////////////
    arma::mat33 tei(const double &time);

    ///////////////////////////////////////////////////////////////////////////////
    // Returns the T.M. of geographic wrt earth coordinates, TGE
    // spherical Earth only
    //
    // Parameter input
    //    lon = geographic longitude - rad
    //    lat = geographic latitude - rad
    //
    // 010628 Created by Peter H Zipfel
    ///////////////////////////////////////////////////////////////////////////////
    arma::mat33 tge(const double &lon, const double &lat);

    ////////////////////////////////////////////////////////////////////////////////
    // Returns the T.M. of geographic (geocentric) wrt inertial
    // using the WGS 84 reference ellipsoid
    // Reference: Britting,K.R."Inertial Navigation Systems Analysis",
    // pp.45-49, Wiley, 1971
    //
    // Return output
    //    TGI(3x3) = T.M.of geographic wrt inertial coord - ND
    // Parameter input
    //    lon = geodetic longitude - rad
    //    lat = geodetic latitude - rad
    //    alt = altitude above ellipsoid - m
    //
    // 030414 Created from FORTRAN by Peter H Zipfel
    // 170121 Create Armadillo Version by soncyang
    ///////////////////////////////////////////////////////////////////////////////
    arma::mat33 tgi84(const double &lon,
                      const double &lat,
                      const double &alt,
                      const double &time);

    ///////////////////////////////////////////////////////////////////////////////
    // Returns the transformation matrix of inertial wrt perifocal coordinates
    //
    // Return output
    //    TIP = TM of inertial wrt perifocal
    // Parameter input
    //    incl = inclination of orbital wrt equatorial plane - rad
    //    lon_anode = celestial longitude of the ascending node - rad
    //    arg_peri = argument of periapsis (ascending node to periapsis) - rad
    //
    // 040510 Created by Peter H Zipfel
    ///////////////////////////////////////////////////////////////////////////////
    arma::mat33 tip(const double &incl,
                    const double &lon_anode,
                    const double &arg_peri);
}
#endif  // __CAD_UTIL_HH__
