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
 * \file global_constants.hh
 *
 * \brief Defines all global constant parameters for HYPER simulation
 */
const double __REARTH                = 6378000;  ///< mean earth radius - m
const double __WEII1                 = 1.20824e-7;
const double __WEII2                 = -2.944e-9;
const double __WEII3                 = 7.292115e-5;   ///< angular rotation of earth - rad/s
const double __AGRAV                 = 9.80665;  ///< standard value of gravity acceleration - m/s^2
const double __G                     = 6.673e-11;  ///< universal gravitational constant - Nm^2/kg^2
const double __EARTH_MASS            = 5.973332e24;  ///< mass of the earth - kg
const double __GM                    = 3.9860044e14;  ///< gravitational parameter=G*EARTH_MASS - m^3/s^2
const double __C20                   = -4.8416685e-4;  ///< second degree zonal gravitational coefficient - ND
const double __FLATTENING            = 1 / 298.257223563;  ///< flattening of the Earth (WGS84) - ND  3.3352810665e-3
const double __SMAJOR_AXIS           = 6378137;  ///< semi-major axis of Earth's ellipsoid (WGS84) - m
const double __GW_CLONG              = 0;  ///< Greenwich celestial longitude at start of simulation - rad
const double __RGAS                  = 287.053;     ///< ideal gas constant - J/(K*kg;=N*m/(K*kg)
const double __KBOLTZ                = 1.38e-23;  ///< Boltzmann's constant - Ws/K
const double ___PI                   = 3.1415926536;  ///< circumference of unit diameter circle
const double __EPS                   = 1.e-10;       ///< machine precision error (type double)
const double __SMALL                 = 1e-7;       ///< small real number
const double __ILARGE                = 9999;         ///< large integer number
const double __LARGE                 = 1e10;       ///< large real number (type double)


/// conversion factors
/**@{*/
const double __RAD                   = 0.0174532925199432;  ///< conversion factor deg->rad
const double __DEG                   = 57.2957795130823;    ///< conversion factor rad->deg
const double __FOOT                  = 3.280834;           ///< conversion factor m->ft
const double __NMILES                = 5.399568e-4;      ///< conversion factor m->nm


/* verify the following array sizes.
 * If too small, dynamic memory allocation will fail!
 */
const double __NROUND6               = 600;  ///< size of 'round6' module-variable array
const double __NHYPER                = 950;   ///< size of 'hyper' module-variable array
const double __NEVENT                = 20;   ///< max number of events
const double __NVAR                  = 50;    ///< max number of variables to be input at every event
const double __NMARKOV               = 20; ///< max number of Markov noise variables

const double __DM_sec2r              = 0.000072722052; /* second to radian */
const double __DM_arcsec2r           = 4.848136811e-6; /* arcsecond to radian */

const double __GMSB                  = 3.490658504e-4;  // ADIS16488 gyro MSB = 0.02d/s
const double __AMSB                  = 0.8 * 0.001 * 9.81;  // ADIS16488 acc MSB = 0.8mg
const double __GLSB                  = 0.01 * (1.0 / 65536.0);
const double __ALSB                  = 0.01 * (1.0 / 65536.0);

/************************************************************************/
/** GPS constellation **/
const double __N_SBF                 = 5;
const double __MAX_SAT               = 32;
const double __MAX_CHAR              = 100;
const double __N_DWRD_SBF            = 10;
const double __CA_SEQ_LEN            = 1023;
const double __N_DWRD                = 60;  //  (__N_SBF + 1) * __N_DWRD_SBF
const double __EPHEM_ARRAY_SIZE      = 13;
const double __SECONDS_IN_HOUR       = 3600;
const double __MAX_CHAN              = 12;
const double __SECONDS_IN_DAY        = 86400;
const double __SPEED_OF_LIGHT        = 2.99792458e8;
const double __LAMBDA_L1             = 0.190293672798365;
const double __POW2_M5               = 0.03125;
const double __POW2_M19              = 1.907348632812500e-6;
const double __POW2_M29              = 1.862645149230957e-9;
const double __POW2_M31              = 4.656612873077393e-10;
const double __POW2_M33              = 1.164153218269348e-10;
const double __POW2_M43              = 1.136868377216160e-13;
const double __POW2_M55              = 2.775557561562891e-17;
const double __POW2_M50              = 8.881784197001252e-016;
const double __POW2_M30              = 9.313225746154785e-010;
const double __POW2_M27              = 7.450580596923828e-009;
const double __POW2_M24              = 5.960464477539063e-008;
const double __WGS84_ECCENTRICITY    = 0.0818191908426;
/****************************************************************/
const double __TVC_DSP_RESOLUTION    = 0.01;
const double __TVC_ROTATION_LIMIT_CNT= 800;

