
#include "rocket/Environment.hh"
#include "sim_services/include/simtime.h"

#include "cad/utility.hh"

#include "cad/env/atmosphere.hh"
#include "cad/env/atmosphere76.hh"
#include "cad/env/atmosphere_nasa2002.hh"
#include "cad/env/atmosphere_weatherdeck.hh"

#include "cad/env/wind.hh"
#include "cad/env/wind_no.hh"
#include "cad/env/wind_tabular.hh"
#include "cad/env/wind_constant.hh"

Environment::Environment(Newton &newt, AeroDynamics &aero, Kinematics &kine, time_management &Time)
    :   newton(&newt), aerodynamics(&aero), kinematics(&kine), time(&Time),
        VECTOR_INIT(GRAVG, 3),
        MATRIX_INIT(TEI, 3, 3)
{
    this->default_data();

    atmosphere = NULL;
    wind       = NULL;
}

Environment::Environment(const Environment& other)
    :   newton(other.newton), aerodynamics(other.aerodynamics), kinematics(other.kinematics),
        time(other.time),
        VECTOR_INIT(GRAVG, 3),
        MATRIX_INIT(TEI, 3, 3)
{
    this->default_data();

    atmosphere = NULL;
    wind       = NULL;

    if(other.atmosphere)
        this->atmosphere = new cad::Atmosphere(*other.atmosphere);
    if(other.wind)
        this->wind = new cad::Wind(*other.wind);

    this->GRAVG = other.GRAVG;
    this->vmach = other.vmach;
    this->pdynmc = other.pdynmc;
    this->dvba = other.dvba;

}

Environment& Environment::operator=(const Environment& other) {
    if(&other == this)
        return *this;

    this->kinematics   = other.kinematics;
    this->newton       = other.newton;
    this->aerodynamics = other.aerodynamics;

    if(other.atmosphere)
        this->atmosphere = new cad::Atmosphere(*other.atmosphere);
    if(other.wind)
        this->wind = new cad::Wind(*other.wind);

    this->GRAVG = other.GRAVG;
    this->vmach = other.vmach;
    this->pdynmc = other.pdynmc;
    this->dvba = other.dvba;

    return *this;
}

Environment::~Environment() {
    if(atmosphere) delete atmosphere;
    if(wind) delete wind;
}

void Environment::initialize() {
    dvba = newton->get_dvbe();
}

void Environment::default_data() {
}

void Environment::atmosphere_use_public() {
    atmosphere = new cad::Atmosphere76();
}

void Environment::atmosphere_use_nasa() {
    atmosphere = new cad::Atmosphere_nasa2002();
}

void Environment::atmosphere_use_weather_deck(char* filename) {
    atmosphere = new cad::Atmosphere_weatherdeck(filename);
}

void Environment::set_no_wind(){
    wind = new cad::Wind_No();
}

void Environment::set_constant_wind(double dvae, double dir, double twind, double vertical_wind){
    wind = new cad::Wind_Constant(dvae, dir, twind, vertical_wind);
}

void Environment::set_tabular_wind(char* filename, double twind, double vertical_wind){
    wind = new cad::Wind_Tabular(filename, twind, vertical_wind);
}

void Environment::set_no_wind_turbulunce(){
    if(wind) wind->disable_turbulance();
}

void Environment::set_wind_turbulunce(double turb_length, double turb_sigma,
                                        double taux1, double taux1d,
                                        double taux2, double taux2d,
                                        double tau,   double gauss_value){
    if(wind) wind->enable_turbulance(turb_length, turb_sigma, taux1, taux1d, taux2, taux2d, tau, gauss_value);
}

void Environment::propagate(double int_step) {
    arma::vec3 VBED = newton->get_VBED();
    arma::vec3 SBII = newton->get_SBII();
    double alt = newton->get_alt();

    arma::mat TBD = kinematics->get_TBD();
    double alppx = kinematics->get_alppx();
    double phipx = kinematics->get_phipx();

    dm_RNP();//Calculate Rotation-Nutation-Precession (ECI to ECEF) Matrix

    this->GRAVG = cad::grav84(SBII, get_rettime());

    atmosphere->set_altitude(alt);

    wind->set_altitude(alt);
    wind->propagate_VAED(int_step);
    wind->apply_turbulance_if_have(int_step, dvba, TBD, alppx, phipx);

    //flight conditionsnorm(VBAD)
    arma::vec3 VBAD = VBED - wind->get_VAED();
    this->dvba = norm(VBAD);

    //mach number
    this->vmach = fabs(this->dvba / atmosphere->get_speed_of_sound());

    //dynamic pressure
    this->pdynmc = 0.5 * atmosphere->get_density() * this->dvba * this->dvba;
}

void Environment::update_diagnostic_attributes(double int_step) {
    this->gravg = get_grav();
    this->tempc = atmosphere->get_temperature_in_kelvin() + 273.16;
}

double Environment::get_press() { return atmosphere->get_pressure(); }
double Environment::get_rho() { return atmosphere->get_density(); }
double Environment::get_tempk() { return atmosphere->get_temperature_in_kelvin(); }

double Environment::get_vmach() { return vmach; }
double Environment::get_pdynmc() { return pdynmc; }
double Environment::get_dvba() { return dvba; }

double Environment::get_grav() { return norm(GRAVG); }

arma::vec3 Environment::get_GRAVG() { return GRAVG; }
arma::vec3 Environment::get_VAED() { return wind->get_VAED(); }

/* Rotation-Nutation-Precession transfor Matrix (ECI to ECEF) */
void Environment::dm_RNP()
{
    /* double We = 7.2921151467E-5; */
    //GPSR gpsr;/* call gpsr function */
    CALDATE utc_caldate;
    GPS tmp_gps;
    unsigned char  i;
    double UTC, UT1;
    arma::mat33 M_rotation;
    arma::mat33 M_nutation;
    arma::mat33 M_precession;
    arma::mat33 M_nut_n_pre;
    double t, t2, t3, thetaA, zetaA, zA;
    double epsilonA, epsilonAP, F,D,omega;
    double temps_sideral(0);
    double L, La, gamma, delta_psi, delta_epsilon;
    double dUT1;
    double s_thetaA, c_thetaA, s_zetaA, c_zetaA, s_zA, c_zA;
    double s_delta_psi, c_delta_psi, s_epsilonA, c_epsilonA, s_epsilonAP, c_epsilonAP;
    double s2_half_delta_psi, s_delta_epsilon, c_delta_epsilon;
    // double DM_Julian_century, DM_w_precessing, DM_sidereal_time;
    // double DM_j2000_wgs84[3][3];
    // double DM_wgs84_j2000[3][3];
    double mjd;/*double GC_swtwopi;*/
    // double p1,p2,p9,p10,p11,p12,p13,p31,p32,p33,p34,p35,p36;/*double bdf;*/
    // double q1,q2,q9,q10,q11,q12,q13,q31,q32,q33,q34,q35,q36;/*double D_UT1_UTC;*/
    int index;


    /*double phi;*/
    /*double GHA0, GHA;*/       /* Greenwich hour angle of the Mean Equinox */
    /*double delta_t;*/

    double nutation_coef[106][9] =
    {
        {  0,  0,  0,  0, 1, -17.1996, -0.01742,  9.2025,  0.00089 },
        {  0,  0,  2, -2, 2, -1.3187,  -0.00016,  0.5736, -0.00031 },
        {  0,  0,  2,  0, 2, -0.2274,  -0.00002,  0.0977, -0.00005 },
        {  0,  0,  0,  0, 2,  0.2062,   0.00002, -0.0895,  0.00005 },
        {  0,  1,  0,  0, 0,  0.1426,  -0.00034,  0.0054, -0.00001 },
        {  1,  0,  0,  0, 0,  0.0712,   0.00001, -0.0007,  0.00000 },
        {  0,  1,  2, -2, 2, -0.0517,   0.00012,  0.0224, -0.00006 },
        {  0,  0,  2,  0, 1, -0.0386,  -0.00004,  0.0200,  0.00000 },
        {  1,  0,  2,  0, 2, -0.0301,   0.00000,  0.0129, -0.00001 },
        {  0, -1,  2, -2, 2,  0.0217,  -0.00005, -0.0095,  0.00003 },
        {  1,  0,  0, -2, 0, -0.0158,   0.00000, -0.0001,  0.00000 },
        {  0,  0,  2, -2, 1,  0.0129,   0.00001, -0.0070,  0.00000 },
        { -1,  0,  2,  0, 2,  0.0123,   0.00000, -0.0053,  0.00000 },
        {  1,  0,  0,  0, 1,  0.0063,   0.00001, -0.0033,  0.00000 },
        {  0,  0,  0,  2, 0,  0.0063,   0.00000, -0.0002,  0.00000 },
        { -1,  0,  2,  2, 2, -0.0059,   0.00000,  0.0026,  0.00000 },
        { -1,  0,  0,  0, 1, -0.0058,  -0.00001,  0.0032,  0.00000 },
        {  1,  0,  2,  0, 1, -0.0051,   0.00000,  0.0027,  0.00000 },
        {  2,  0,  0, -2, 0,  0.0048,   0.00000,  0.0001,  0.00000 },
        { -2,  0,  2,  0, 1,  0.0046,   0.00000, -0.0024,  0.00000 },
        {  0,  0,  2,  2, 2, -0.0038,   0.00000,  0.0016,  0.00000 },
        {  2,  0,  2,  0, 2, -0.0031,   0.00000,  0.0013,  0.00000 },
        {  2,  0,  0,  0, 0,  0.0029,   0.00000, -0.0001,  0.00000 },
        {  1,  0,  2, -2, 2,  0.0029,   0.00000, -0.0012,  0.00000 },
        {  0,  0,  2,  0, 0,  0.0026,   0.00000, -0.0001,  0.00000 },
        {  0,  0,  2, -2, 0, -0.0022,   0.00000,  0.0000,  0.00000 },
        { -1,  0,  2,  0, 1,  0.0021,   0.00000, -0.0010,  0.00000 },
        {  0,  2,  0,  0, 0,  0.0017,  -0.00001,  0.0000,  0.00000 },
        {  0,  2,  2, -2, 2, -0.0016,   0.00001,  0.0007,  0.00000 },
        { -1,  0,  0,  2, 1,  0.0016,   0.00000, -0.0008,  0.00000 },
        {  0,  1,  0,  0, 1, -0.0015,   0.00000,  0.0009,  0.00000 },
        {  1,  0,  0, -2, 1, -0.0013,   0.00000,  0.0007,  0.00000 },
        {  0, -1,  0,  0, 1, -0.0012,   0.00000,  0.0006,  0.00000 },
        {  2,  0, -2,  0, 0,  0.0011,   0.00000,  0.0000,  0.00000 },
        { -1,  0,  2,  2, 1, -0.0010,   0.00000,  0.0005,  0.00000 },
        {  1,  0,  2,  2, 2, -0.0008,   0.00000,  0.0003,  0.00000 },
        {  0, -1,  2,  0, 2, -0.0007,   0.00000,  0.0003,  0.00000 },
        {  0,  0,  2,  2, 1, -0.0007,   0.00000,  0.0003,  0.00000 },
        {  1,  1,  0, -2, 0, -0.0007,   0.00000,  0.0000,  0.00000 },
        {  0,  1,  2,  0, 2,  0.0007,   0.00000, -0.0003,  0.00000 },
        { -2,  0,  0,  2, 1, -0.0006,   0.00000,  0.0003,  0.00000 },
        {  0,  0,  0,  2, 1, -0.0006,   0.00000,  0.0003,  0.00000 },
        {  2,  0,  2, -2, 2,  0.0006,   0.00000, -0.0003,  0.00000 },
        {  1,  0,  0,  2, 0,  0.0006,   0.00000,  0.0000,  0.00000 },
        {  1,  0,  2, -2, 1,  0.0006,   0.00000, -0.0003,  0.00000 },
        {  0,  0,  0, -2, 1, -0.0005,   0.00000,  0.0003,  0.00000 },
        {  0, -1,  2, -2, 1, -0.0005,   0.00000,  0.0003,  0.00000 },
        {  2,  0,  2,  0, 1, -0.0005,   0.00000,  0.0003,  0.00000 },
        {  1, -1,  0,  0, 0,  0.0005,   0.00000,  0.0000,  0.00000 },
        {  1,  0,  0, -1, 0, -0.0004,   0.00000,  0.0000,  0.00000 },
        {  0,  0,  0,  1, 0, -0.0004,   0.00000,  0.0000,  0.00000 },
        {  0,  1,  0, -2, 0, -0.0004,   0.00000,  0.0000,  0.00000 },
        {  1,  0, -2,  0, 0,  0.0004,   0.00000,  0.0000,  0.00000 },
        {  2,  0,  0, -2, 1,  0.0004,   0.00000, -0.0002,  0.00000 },
        {  0,  1,  2, -2, 1,  0.0004,   0.00000, -0.0002,  0.00000 },
        {  1,  1,  0,  0, 0, -0.0003,   0.00000,  0.0000,  0.00000 },
        {  1, -1,  0, -1, 0, -0.0003,   0.00000,  0.0000,  0.00000 },
        { -1, -1,  2,  2, 2, -0.0003,   0.00000,  0.0001,  0.00000 },
        {  0, -1,  2,  2, 2, -0.0003,   0.00000,  0.0001,  0.00000 },
        {  1, -1,  2,  0, 2, -0.0003,   0.00000,  0.0001,  0.00000 },
        {  3,  0,  2,  0, 2, -0.0003,   0.00000,  0.0001,  0.00000 },
        { -2,  0,  2,  0, 2, -0.0003,   0.00000,  0.0001,  0.00000 },
        {  1,  0,  2,  0, 0,  0.0003,   0.00000,  0.0000,  0.00000 },
        { -1,  0,  2,  4, 2, -0.0002,   0.00000,  0.0001,  0.00000 },
        {  1,  0,  0,  0, 2, -0.0002,   0.00000,  0.0001,  0.00000 },
        { -1,  0,  2, -2, 1, -0.0002,   0.00000,  0.0001,  0.00000 },
        {  0, -2,  2, -2, 1, -0.0002,   0.00000,  0.0001,  0.00000 },
        { -2,  0,  0,  0, 1, -0.0002,   0.00000,  0.0001,  0.00000 },
        {  2,  0,  0,  0, 1,  0.0002,   0.00000, -0.0001,  0.00000 },
        {  3,  0,  0,  0, 0,  0.0002,   0.00000,  0.0000,  0.00000 },
        {  1,  1,  2,  0, 2,  0.0002,   0.00000, -0.0001,  0.00000 },
        {  0,  0,  2,  1, 2,  0.0002,   0.00000, -0.0001,  0.00000 },
        {  1,  0,  0,  2, 1, -0.0001,   0.00000,  0.0000,  0.00000 },
        {  1,  0,  2,  2, 1, -0.0001,   0.00000,  0.0001,  0.00000 },
        {  1,  1,  0, -2, 1, -0.0001,   0.00000,  0.0000,  0.00000 },
        {  0,  1,  0,  2, 0, -0.0001,   0.00000,  0.0000,  0.00000 },
        {  0,  1,  2, -2, 0, -0.0001,   0.00000,  0.0000,  0.00000 },
        {  0,  1, -2,  2, 0, -0.0001,   0.00000,  0.0000,  0.00000 },
        {  1,  0, -2, -2, 0, -0.0001,   0.00000,  0.0000,  0.00000 },
        {  1,  0, -2,  2, 0, -0.0001,   0.00000,  0.0000,  0.00000 },
        {  1,  0,  2, -2, 0, -0.0001,   0.00000,  0.0000,  0.00000 },
        {  1,  0,  0, -4, 0, -0.0001,   0.00000,  0.0000,  0.00000 },
        {  2,  0,  0, -4, 0, -0.0001,   0.00000,  0.0000,  0.00000 },
        {  0,  0,  2,  4, 2, -0.0001,   0.00000,  0.0000,  0.00000 },
        {  0,  0,  2, -1, 2, -0.0001,   0.00000,  0.0000,  0.00000 },
        { -2,  0,  2,  4, 2, -0.0001,   0.00000,  0.0001,  0.00000 },
        {  2,  0,  2,  2, 2, -0.0001,   0.00000,  0.0000,  0.00000 },
        {  0, -1,  2,  0, 1, -0.0001,   0.00000,  0.0000,  0.00000 },
        {  0,  0, -2,  0, 1, -0.0001,   0.00000,  0.0000,  0.00000 },
        {  0,  0,  4, -2, 2,  0.0001,   0.00000,  0.0000,  0.00000 },
        {  0,  1,  0,  0, 2,  0.0001,   0.00000,  0.0000,  0.00000 },
        {  1,  1,  2, -2, 2,  0.0001,   0.00000, -0.0001,  0.00000 },
        {  3,  0,  2, -2, 2,  0.0001,   0.00000,  0.0000,  0.00000 },
        { -2,  0,  2,  2, 2,  0.0001,   0.00000, -0.0001,  0.00000 },
        { -1,  0,  0,  0, 2,  0.0001,   0.00000, -0.0001,  0.00000 },
        {  0,  0, -2,  2, 1,  0.0001,   0.00000,  0.0000,  0.00000 },
        {  0,  1,  2,  0, 1,  0.0001,   0.00000,  0.0000,  0.00000 },
        { -1,  0,  4,  0, 2,  0.0001,   0.00000,  0.0000,  0.00000 },
        {  2,  1,  0, -2, 0,  0.0001,   0.00000,  0.0000,  0.00000 },
        {  2,  0,  0,  2, 0,  0.0001,   0.00000,  0.0000,  0.00000 },
        {  2,  0,  2, -2, 1,  0.0001,   0.00000, -0.0001,  0.00000 },
        {  2,  0, -2,  0, 1,  0.0001,   0.00000,  0.0000,  0.00000 },
        {  1, -1,  0, -2, 0,  0.0001,   0.00000,  0.0000,  0.00000 },
        { -1,  0,  0,  1, 1,  0.0001,   0.00000,  0.0000,  0.00000 },
        { -1, -1,  0,  2, 1,  0.0001,   0.00000,  0.0000,  0.00000 },
        {  0,  1,  0,  1, 0,  0.0001,   0.00000,  0.0000,  0.00000 }
    };

    /*------------------------------------------------------------------ */
    /* --------------- Interface to Global Variable ------------*/
    /*------------------------------------------------------------------ */

    /*------------------------------------------------------------- */
    /*--------------- Calculate the UTC time -------------*/
    /*------------------------------------------------------------- */
    /* GPS time converted from GPS format to YYYY/MM/DD/MM/SS */
    /* Correction for time difference btwn GPS & UTC is applied implicitly */

    // tmp_gps.Week  = time->gpstime.Week;
    // tmp_gps.SOW  = time->gpstime.SOW;
    time->gps_to_utc(&(time->gpstime), &utc_caldate);   /* leap second is considered */

    UTC = utc_caldate.Hour * 3600.0 + utc_caldate.Min * 60.0 + utc_caldate.Sec;

    /*** Cynthia

    GC_swtwopi=PI*2;
    mjd = DM_Julian_Date - 2400000.5;
    bdf = 2000.0 + (mjd - 51544.03) / 365.2422;

    D_UT1_UTC= -0.0189 - 0.00085*(mjd-55351)-0.022*sin(GC_swtwopi*bdf)+0.012*cos(GC_swtwopi*bdf)+0.006*sin(2.0*GC_swtwopi*bdf)-0.007*cos(2.0*GC_swtwopi*bdf);

    ***/

    /* printf("D_UT1_UTC=%f\n",D_UT1_UTC);  */  /* - 0.057166 */


    index = (int) (time->Julian_Date - 2400000.5 - 55197.00);   /* get dUT1 = Ut1 - UT from table*/
    if ((index >= 0) && (index < Max_DM_UT1_UT_Index))      /*MJD = 55197.00 (1-1-2010)~ 56150.00(8-11-2012) */
    {
        dUT1 = DM_UT1_UT[index];
    }
    else
    {
        dUT1 = -0.008853655954360;  /* mean value during 19760519~20120811, FSW: dUT1 = 0.4; */
        /* mjd = DM_Julian_Date - 2400000.5; */
        /* dUT1 = delta_UT1(mjd); */
        /* printf("MJD=%f   dUT1=%f\n", mjd, dUT1); */
    }

    UT1 = UTC + dUT1;

    /*** for RNP matrix verification
    printf("DM_time_env = %f\n", DM_time);
    printf("DM_Julian_Date = %20.15f\n", DM_Julian_Date);
    printf("UT1 = %20.15f\n", UT1);
    printf("UTC = %20.15f\n", UTC);
    ***/

    /*----------------------------------------------------------- */
    /*-------------- Precession Matrix  ------------------- */
    /*----------------------------------------------------------- */
    t = (time->Julian_Date - 2451545.0) / 36525.0;  /* J2000.5 : Julian Day is 2451545, unit in day */

    DM_Julian_century = t;  /* elapsed century since J2000.5 */

    t2 = t * t;
    t3 = t * t * t;

    thetaA = 2004.3109 * t - 0.42665 * t2 - 0.041833 * t3; /* unit : arcsec */
    zetaA  = 2306.2181 * t + 0.30188 * t2 + 0.017998 * t3; /* unit : arcsec */
    zA     = 2306.2181 * t + 1.09468 * t2 + 0.018203 * t3; /* unit : arcsec */

    s_thetaA = sin(thetaA * DM_arcsec2r);
    c_thetaA = cos(thetaA * DM_arcsec2r);
    s_zetaA  = sin(zetaA  * DM_arcsec2r);
    c_zetaA  = cos(zetaA  * DM_arcsec2r);
    s_zA     = sin(zA * DM_arcsec2r);
    c_zA     = cos(zA * DM_arcsec2r);

    M_precession(0,0) = -s_zA * s_zetaA + c_zA * c_thetaA * c_zetaA;
    M_precession(0,1) = -s_zA * c_zetaA - c_zA * c_thetaA * s_zetaA;
    M_precession(0,2) = -c_zA * s_thetaA;
    M_precession(1,0) =  c_zA * s_zetaA + s_zA * c_thetaA * c_zetaA;
    M_precession(1,1) =  c_zA * c_zetaA - s_zA * c_thetaA * s_zetaA;
    M_precession(1,2) = -s_zA * s_thetaA;
    M_precession(2,0) =  s_thetaA * c_zetaA;
    M_precession(2,1) = -s_zetaA  * s_thetaA;
    M_precession(2,2) =  c_thetaA;

    /*------------------------------------------------------ */
    /*------------- Nutation Matrix ---------------------*/
    /*------------------------------------------------------ */

    /*** Original Algorithm ***/
    /* IAU-80 nutation */
    epsilonA = (84381.448 - 46.815 * t - 0.00059 * t2 + 0.001813 * t3) * DM_arcsec2r; /* unit: radian */

    L       = (485866.7330 + 1717915922.633 * t + 31.310 * t2 + 0.064 * t3) * DM_arcsec2r; /* unit: radian */
    La      = (1287099.804 + 129596581.2240 * t - 0.5770 * t2 - 0.012 * t3) * DM_arcsec2r; /* unit: radian */
    F       = (335778.8770 + 1739527263.137 * t - 13.257 * t2 + 0.011 * t3) * DM_arcsec2r; /* unit: radian */
    D       = (1072261.307 + 1602961601.328 * t - 6.8910 * t2 + 0.019 * t3) * DM_arcsec2r; /* unit: radian */
    omega   = (450160.2800 -    6962890.539 * t + 7.4550 * t2 + 0.008 * t3) * DM_arcsec2r; /* unit: radian */

    delta_psi     = 0.0;
    delta_epsilon = 0.0;

    for (i = 0; i < 106; i++)
    {
        gamma = nutation_coef[i][0] * L + nutation_coef[i][1] * La + nutation_coef[i][2] * F +
                nutation_coef[i][3] * D + nutation_coef[i][4] * omega;                            /* unit: radian */
        delta_psi     = delta_psi + (nutation_coef[i][5] + nutation_coef[i][6] * t) * sin(gamma); /* unit: arcsec */
        delta_epsilon = delta_epsilon + (nutation_coef[i][7] + nutation_coef[i][8] * t) * cos(gamma); /* unit: arcsec */
    }

    /* delta_psi      = delta_psi     * sec2r; */ /* unit: radian */
    /* delta_epsilon = delta_epsilon * sec2r; */
    epsilonAP  = epsilonA + delta_epsilon * DM_arcsec2r;    /* unit: radian */

    s_delta_psi       = sin(delta_psi * DM_arcsec2r);
    c_delta_psi       = cos(delta_psi * DM_arcsec2r);
    s2_half_delta_psi = sin(delta_psi/2.0 * DM_arcsec2r) * sin(delta_psi/2.0 * DM_arcsec2r);
    s_epsilonA        = sin(epsilonA);
    c_epsilonA        = cos(epsilonA);
    s_delta_epsilon   = sin(delta_epsilon * DM_arcsec2r);
    c_delta_epsilon   = cos(delta_epsilon * DM_arcsec2r);
    s_epsilonAP       = sin(epsilonAP);
    c_epsilonAP       = cos(epsilonAP);

    M_nutation(0,0) =  c_delta_psi;
    M_nutation(0,1) = -s_delta_psi * c_epsilonA;
    M_nutation(0,2) = -s_delta_psi * s_epsilonA;
    M_nutation(1,0) =   c_epsilonAP * s_delta_psi;
    M_nutation(1,1) =  c_delta_epsilon - 2 * s2_half_delta_psi * c_epsilonA * c_epsilonAP;
    M_nutation(1,2) = -s_delta_epsilon - 2 * s2_half_delta_psi * s_epsilonA * c_epsilonAP;
    M_nutation(2,0) =  s_epsilonAP * s_delta_psi;
    M_nutation(2,1) =  s_delta_epsilon - 2 * s2_half_delta_psi * c_epsilonA * s_epsilonAP;
    M_nutation(2,2) =   c_delta_epsilon - 2 * s2_half_delta_psi * s_epsilonA * s_epsilonAP;


    // if (Nut_Algo == 2)
    // {
    //  /*** Cynthia's algorithm ***/
    //  /*D'apres Lieske */
    //  epsilonA = (84381.448 - 46.815 * t - 0.00059 * t2 + 0.001813 * t3) * DM_arcsec2r;

    //  /*D'apres Van Flandern */
    //  L       = (485866.7330 + 1717915922.633 * t + 31.310 * t2 + 0.064 * t3) * DM_arcsec2r;
    //  La      = (1287099.804 + 129596581.2240 * t - 0.5770 * t2 - 0.012 * t3) * DM_arcsec2r;
    //  F       = (335778.8770 + 1739527263.137 * t - 13.257 * t2 + 0.011 * t3) * DM_arcsec2r;
    //  D       = (1072261.307 + 1602961601.328 * t - 6.8910 * t2 + 0.019 * t3) * DM_arcsec2r;
    //  omega   = (450160.2800 -    6962890.539 * t + 7.4550 * t2 + 0.008 * t3) * DM_arcsec2r;


    //  /* seuls les coefficients -- translate "only the coefficients"  */
    //  p1=-171996-174.2*t;
    //  p2=2062+0.2*t;
    //  p9=-13187-1.6*t;
    //  p10=1426-3.4*t;
    //  p11=-517+1.2*t;
    //  p12=217-0.5*t;
    //  p13=129+0.1*t;
    //  p31=-2274-0.2*t;
    //  p32=712+0.1*t;
    //  p33=-386-0.4*t;
    //  p34=-301;
    //  p35=-158;
    //  p36=123;

    //  q1=92025+8.9*t;
    //  q2=-895+0.5*t;
    //  q9=5736-3.1*t;
    //  q10=54-0.1*t;
    //  q11=224-0.6*t;
    //  q12=-95+0.3*t;
    //  q13=-70;
    //  q31=977-0.5*t;
    //  q32=-7;
    //  q33=200;
    //  q34=129-0.1*t;
    //  q35=-1;
    //  q36=-53;

    //  delta_psi     = p1*sin(omega)+p2*sin(2*omega)+p9*sin(2*F-2*D+2*omega)+p10*sin(La)+p11*sin(La+2*F-2*D+2*omega)
    //                 +p12*sin(-La+2*F-2*D+2*omega)+p13*sin(2*F-2*D+omega)+p31*sin(2*F+2*omega)
    //                 +p32*sin(L)+p33*sin(2*F+omega)+p34*sin(L+2*F+2*omega)+p35*sin(L-2*D)+p36*sin(-L+2*F+2*omega);

    //  delta_epsilon = q1*cos(omega)+q2*cos(2*omega)+q9*cos(2*F-2*D+2*omega)+q10*cos(La)+q11*cos(La+2*F-2*D+2*omega)
    //                  +q12*cos(-La+2*F-2*D+2*omega)+q13*cos(2*F-2*D+omega)+q31*cos(2*F+2*omega)+q32*cos(L)
    //                  +q33*cos(2*F+omega)+q34*cos(L+2*F+2*omega)+q35*cos(L-2*D)+q36*cos(-L+2*F+2*omega);

    //  delta_psi     = 0.0001 * delta_psi     * DM_arcsec2r;
    //  delta_epsilon = 0.0001 * delta_epsilon * DM_arcsec2r;
    //   epsilonAP  = epsilonA + delta_epsilon * DM_arcsec2r;


    //  /* matrice of nutation */
    //  M_nutation[0][0] =  1- 0.5 * delta_psi * delta_psi;
    //  M_nutation[0][1] = -delta_psi * cos(epsilonA);
    //  M_nutation[0][2] = -delta_psi * sin(epsilonA);
    //  M_nutation[1][0] =  delta_psi * cos(epsilonA) - delta_epsilon * delta_psi * sin(epsilonA);
    //  M_nutation[1][1] =  1 - 0.5 * delta_epsilon * delta_epsilon - 0.5 * delta_psi * delta_psi * cos(epsilonA) * cos(epsilonA);
    //  M_nutation[1][2] = -delta_epsilon - 0.5 * delta_psi * delta_psi * sin(epsilonA) * cos(epsilonA);
    //  M_nutation[2][0] =  delta_psi * sin(epsilonA) + delta_epsilon * delta_psi * cos(epsilonA);
    //  M_nutation[2][1] =  delta_epsilon - 0.5 * delta_psi * delta_psi * sin(epsilonA) * cos(epsilonA);
    //  M_nutation[2][2] =  1 - 0.5 * delta_epsilon * delta_epsilon - 0.5 * delta_psi * delta_psi * sin(epsilonA) * sin(epsilonA);
    // }


    // if (Nut_Algo == 3)
    // {
    //  mjd = DM_Julian_Date - 2451545.0;
    //  Nutation(mjd, &delta_psi, &delta_epsilon, &epsilonA);
    //  /* matrice of nutation */
    //  M_nutation[0][0] =  1- 0.5 * delta_psi * delta_psi;
    //  M_nutation[0][1] = -delta_psi * cos(epsilonA);
    //  M_nutation[0][2] = -delta_psi * sin(epsilonA);
    //  M_nutation[1][0] =  delta_psi * cos(epsilonA) - delta_epsilon * delta_psi * sin(epsilonA);
    //  M_nutation[1][1] =  1 - 0.5 * delta_epsilon * delta_epsilon - 0.5 * delta_psi * delta_psi * cos(epsilonA) * cos(epsilonA);
    //  M_nutation[1][2] = -delta_epsilon - 0.5 * delta_psi * delta_psi * sin(epsilonA) * cos(epsilonA);
    //  M_nutation[2][0] =  delta_psi * sin(epsilonA) + delta_epsilon * delta_psi * cos(epsilonA);
    //  M_nutation[2][1] =  delta_epsilon - 0.5 * delta_psi * delta_psi * sin(epsilonA) * cos(epsilonA);
    //  M_nutation[2][2] =  1 - 0.5 * delta_epsilon * delta_epsilon - 0.5 * delta_psi * delta_psi * sin(epsilonA) * sin(epsilonA);
    // }

    /*----------------------------------------------------------- */
    /*-------- Matrice of Nutation * Precession --------*/
    /*----------------------------------------------------------- */
    M_nut_n_pre = M_nutation * M_precession;

    /*----------------------------------------------------------- */
    /*------------------- Rotation Matrix --------------------*/
    /*----------------------------------------------------------- */

    /* w* = Rotation Rate i n Precessing Reference Frame */
    /* unit : (Radians/Second) */
    /* source: DMA TECHNICAL REPORT TR8350.2-a - (Second Printing - 1 December 1987) - Appendix */
    /* http://earth-info.nga.mil/GandG/publications/historic/historic.html */

    DM_w_precessing = 7.2921158553e-5 + 4.3e-15 * t; /* refer to Vallado */


    /*  temps_sideral =  */ /* (DM_w_precessing * UT1) + */ /* unit: radian */
    /*                  UT1 * sec2r +    */ /* unit: radian */
    /*                  (24110.54841 + 8640184.812866 * t + 0.093104 * t2 - 0.0000062 * t3) * sec2r  */
    /*                  + delta_psi * cos(epsilonA);  */

    temps_sideral = UT1 +  (24110.54841 + 8640184.812866 * t + 0.093104 * t2 - 0.0000062 * t3);

    /* printf("DM_time=%f, t=%20.15f, t2=%20.15f, t3=%20.15f, UT1=%20.15f, UTC=%20.15f, dUT1=%20.15f, temps_sideral_in_sec=%30.20f, ",
           DM_time, t, t2, t3, UT1, UTC, dUT1, temps_sideral); -- printf for Prof. Hsiao - 20111123 */

    /*printf("Week=%d, SOW=%20.15f, Hour=%d, Min=%d, Sec=%23.20f, ",
           DM_current_gps_time.Week, DM_current_gps_time.SOW, caldate.Hour, caldate.Min, caldate.Sec); -- printf for Prof. Hsiao - 20111123 */

    /*  temps_sideral = 67310.54841 + 3164400184.812866 * t + 0.093104 * t2 - 0.0000062 * t3;*/
    /*  temps_sideral = dm_fmod(temps_sideral, 86400.0);*/

    temps_sideral = temps_sideral * DM_sec2r + delta_psi * cos(epsilonA) * DM_arcsec2r; /* unit: radian */
    /* Prof. Hsiao's Simulink program didn't multiply DM_arcsec2r, therefore the result differs from C ode's result. - 2011/11/24 */


    /* printf("temps_sideral_in_rad=%30.20f\n", temps_sideral);  -- printf for Prof. Hsiao - 20111123 */

    // if (Nut_Algo == 2)
    // {
    //  temps_sideral = temps_sideral * DM_sec2r + delta_psi * cos(epsilonA); /* unit: radian */
    // }

    DM_sidereal_time = temps_sideral;

    /***
    if (DM_fctr == 1)
    printf("T = %f, temps_sideral = %20.15f, century = %20.15f\n", DM_time, temps_sideral, t);
    ***/

    /*  temps_sideral = (UT1 + 24110.54841 + 8640184.812866 * t + 0.093104 * t2 - 0.0000062 * t3) * sec2r; */
    /*  temps_sideral = temps_sideral + delta_psi * cos(epsilonA); */
    /*  temps_sideral = dm_fmod(temps_sideral, 2.0*PI); */

    /***
    if (DM_time >= 7000)
    {
    printf("\nDM_UT1=%20.15f\n", UT1);
    printf("DM Week=%d, SOW=%20.15f\n", DM_current_gps_time.Week, DM_current_gps_time.SOW);
    printf("DM_temps_sideral=%20.15f\n",temps_sideral);
    printf("DM_Julian_Date = %20.15f\n", DM_Julian_Date);
    printf("DM_Julian_century=%20.15f\n\n", DM_Julian_century);
    }
    ***/

    M_rotation(0,0) = cos(temps_sideral);
    M_rotation(0,1) = sin(temps_sideral);
    M_rotation(0,2) = 0.0;
    M_rotation(1,0) = -sin(temps_sideral);
    M_rotation(1,1) = cos(temps_sideral);
    M_rotation(1,2) = 0.0;
    M_rotation(2,0) = 0.0;
    M_rotation(2,1) = 0.0;
    M_rotation(2,2) = 1.0;

    /*** Gene Brownd's way
    GHA0 = delta_psi * cos(epsilonA);
    delta_t = dm_fmod(UT1, 86400.0);
    GHA = (delta_t + 24110.54841 + 8640184.812866 * t + 0.093104 * t2 - 0.0000062 * t3) * sec2r;
    GHA = dm_fmod((GHA + GHA0), (2.0 * PI));

    M_rotation[0][0] = cos(GHA);
    M_rotation[0][1] = sin(GHA);
    M_rotation[0][2] = 0.0;
    M_rotation[1][0] = -sin(GHA);
    M_rotation[1][1] = cos(GHA);
    M_rotation[1][2] = 0.0;
    M_rotation[2][0] = 0.0;
    M_rotation[2][1] = 0.0;
    M_rotation[2][2] = 1.0;

    ***/

    /*-------------------------------------------------------------------------- */
    /* Matrice of WGS84_J2000 : From J2000 to WGS84   */
    /*-------------------------------------------------------------------------- */
    /*
    M_nut_n_pre[0][0] = 1.0;
    M_nut_n_pre[0][1] = 0.0;
    M_nut_n_pre[0][2] = 0.0;
    M_nut_n_pre[1][0] = 0.0;
    M_nut_n_pre[1][1] = 1.0;
    M_nut_n_pre[1][2] = 0.0;
    M_nut_n_pre[2][0] = 0.0;
    M_nut_n_pre[2][1] = 0.0;
    M_nut_n_pre[2][2] = 1.0;
    */

    this->TEI = M_rotation * M_nut_n_pre;
    /*
        for (i = 0; i < 3; i++)
        {
            for (j = 0; j < 3; j++)
            {
                DM_j2000_wgs84[i][j] = M_rotation[i][0] * M_nut_n_pre[0][j]
                                     + M_rotation[i][1] * M_nut_n_pre[1][j]
                                     + M_rotation[i][2] * M_nut_n_pre[2][j];
            }
        }
    */

    /**
    if (DM_time >= 7000)
    {
    printf("DM__RNP =");
    for (i=0;i<3;i++) for (j=0;j<3;j++) printf("%20.15f ,", DM_j2000_wgs84[i][j]); printf("\n");
    }
    **/

}  /* End of dm_RNP() */
