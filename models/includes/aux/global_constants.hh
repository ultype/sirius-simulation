/********************************* TRICK HEADER *******************************
PURPOSE:
      (Global Constants migrated from CADAC++)
LIBRARY DEPENDENCY:
      ()
PROGRAMMERS:
      ((() () () () ))
ICG: (No)
*******************************************************************************/

/**
 * \file global_constants.hpp
 *
 * \brief Defines all global constant parameters for HYPER simulation
 */

#ifndef global_constants__HPP
#define global_constants__HPP

/// global constants to be used in the simulation

/// physical constants
/**@{*/
double const REARTH = 6378000;  ///< mean earth radius - m
double const WEII1 = 1.20824e-7;
double const WEII2 = -2.944e-9 ;
double const WEII3 = 7.292115e-5;   ///< angular rotation of earth - rad/s
double const AGRAV =
    9.80675445;              ///< standard value of gravity acceleration - m/s^2
double const G = 6.673e-11;  ///< universal gravitational constant - Nm^2/kg^2
double const EARTH_MASS = 5.973332e24;  ///< mass of the earth - kg
double const GM =
    3.9860044e14;  ///< gravitational parameter=G*EARTH_MASS - m^3/s^2
double const C20 =
    -4.8416685e-4;  ///< second degree zonal gravitational coefficient - ND
double const FLATTENING =
    3.33528106e-3;  ///< flattening of the Earth (WGS84) - ND
double const SMAJOR_AXIS =
    6378137;  ///< semi-major axis of Earth's ellipsoid (WGS84) - m
double const GW_CLONG =
    0;  ///< Greenwich celestial longitude at start of simulation - rad
double const RGAS = 287.053;     ///< ideal gas constant - J/(K*kg)=N*m/(K*kg)
double const KBOLTZ = 1.38e-23;  ///< Boltzmann's constant - Ws/K
// numerical constants
double const PI = 3.1415926536;  ///< circumference of unit diameter circle
double const EPS = 1.e-10;       ///< machine precision error (type double)
double const SMALL = 1e-7;       ///< small real number
int const ILARGE = 9999;         ///< large integer number
double const LARGE = 1e10;       ///< large real number (type double)
/**@}*/

/// conversion factors
/**@{*/
double const RAD = 0.0174532925199432;  ///< conversion factor deg->rad
double const DEG = 57.2957795130823;    ///< conversion factor rad->deg
double const FOOT = 3.280834;           ///< conversion factor m->ft
double const NMILES = 5.399568e-4;      ///< conversion factor m->nm
                                        /**@}*/

/* verify the following array sizes.
 * If too small, dynamic memory allocation will fail!
 */
int const NROUND6 = 600;  ///< size of 'round6' module-variable array
int const NHYPER = 950;   ///< size of 'hyper' module-variable array
int const NEVENT = 20;    ///< max number of events
int const NVAR = 50;     ///< max number of variables to be input at every event
int const NMARKOV = 20;  ///< max number of Markov noise variables
/**@}*/


/** UTC Time/Julian Day/GPS Time **/
const double DJ00 = 2451545.0;
const double DJC = 36525.0;
const double DAS2R = 4.848136811095359935899141e-6; /* Arcseconds to radians */
const double D2PI = 6.283185307179586476925287; /* 2Pi */
const double DMAS2R = 4.848136811095359935899141e-9; /* Milliarcseconds to radians */
const double TURNAS = 1296000.0; /* Arcseconds in a full circle */
const double JAN61980 = 44244.0;  /* MJD of GPS epoch : Jan. 6 1980 */
const double JAN11901 = 15385.0;  /* MJD of Jan 1, 1901 */
const double SEC_PER_DAY = 86400.0;
const double SEC_PER_WEEK = 604800.0;
const int TAI_GPS = 19;   /* Difference between TAI and GPS */
const int UNIX_MJD = 40587; /* MJD of Unix epoch : 00:00:00 on January 1, 1970 */
const double TT_TAI = 32.184;
const double DM_sec2r    =  0.000072722052; /* second to radian */
const double DM_arcsec2r =  4.848136811e-6; /* arcsecond to radian */
/************************************************************************/
/** GPS constellation **/
const int  N_SBF = 5;
const int  MAX_SAT = 32;
const int  MAX_CHAR = 100;
const int  N_DWRD_SBF = 10;
const int  CA_SEQ_LEN = 1023;
const int  N_DWRD = (N_SBF+1)*N_DWRD_SBF;
const int  EPHEM_ARRAY_SIZE = 13;
const int  TRUE = 1;
const int  FALSE = 0;
const int  SECONDS_IN_HOUR = 3600;
const int  MAX_CHAN = 12;
const int  SECONDS_IN_DAY = 86400;
const double  SPEED_OF_LIGHT = 2.99792458e8;
const double  LAMBDA_L1 = 0.190293672798365;
const double POW2_M5 = 0.03125;
const double POW2_M19 = 1.907348632812500e-6;
const double POW2_M29 = 1.862645149230957e-9;
const double POW2_M31 = 4.656612873077393e-10;
const double POW2_M33 = 1.164153218269348e-10;
const double POW2_M43 = 1.136868377216160e-13;
const double POW2_M55 = 2.775557561562891e-17;
const double POW2_M50 = 8.881784197001252e-016;
const double POW2_M30 = 9.313225746154785e-010;
const double POW2_M27 = 7.450580596923828e-009;
const double POW2_M24 = 5.960464477539063e-008;
const double WGS84_ECCENTRICITY = 0.0818191908426;
/****************************************************************/

#endif
