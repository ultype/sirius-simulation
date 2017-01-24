#include "utility_header.hh"

#include "Ins.hh"
#include "sim_services/include/simtime.h"

void INS::default_data(){
}

///////////////////////////////////////////////////////////////////////////////
// INS initialization module
// Member function of class 'Hyper'
// Initializes INS error states using the Cholesky method
//
// mins  = 0 ideal INS (no errors)
//    = 1 space stabilized INS
//
// 030604 Created by Peter H Zipfel
// 081118 Improved initialization, PZi
///////////////////////////////////////////////////////////////////////////////
void INS::initialize(Newton *ntn, _Euler_ *elr, Environment *env, Kinematics *kins, GPS_Receiver *gps){

    newton = ntn;
    euler = elr;
    environment = env;
    kinematics = kins;
    gpsr = gps;

    // local module-variables
    Matrix ESBI(3, 1);
    Matrix EVBI(3, 1);
    Matrix RICI(3, 1);

    //-------------------------------------------------------------------------
    // initialization without INS errors (perfect transfer alignment)
    if (mins == 0) {
        // do nothing
    } else {
        // Initial covariance matrix  (GPS quality)
        // equipped aircraft. Units: meter, meter/sec, milli-rad.
        double PP0[9][9] = {
            20.701,      0.12317,     0.10541,     6.3213E-02,  2.2055E-03,
            1.7234E-03,  1.0633E-03,  3.4941E-02,  -3.5179E-02,

            0.12317,     20.696,      -0.27174,    4.8366E-03,  5.9463E-02,
            -1.3367E-03, -3.4903E-02, 2.6112E-03,  -4.2663E-02,

            0.10541,     -0.27174,    114.12,      5.6373E-04,  -8.3147E-03,
            5.4059E-02,  1.5496E-02,  7.6463E-02,  -3.5302E-03,

            6.3213E-02,  4.8366E-03,  5.6373E-04,  1.9106E-03,  8.0945E-05,
            1.9810E-06,  2.5755E-04,  2.8346E-03,  -5.6482E-04,

            2.2055E-03,  5.9463E-02,  -8.3147E-03, 8.0945E-05,  1.7201E-03,
            -1.5760E-05, -2.8341E-03, 2.6478E-04,  -1.0781E-03,

            1.7234E-03,  -1.3367E-03, 5.4059E-02,  1.9810E-06,  -1.5760E-05,
            3.0070E-03,  4.1963E-04,  -1.3297E-04, 4.1190E-05,

            1.0638E-03,  -3.4903E-02, 1.5496E-02,  2.5755E-04,  -2.8341E-03,
            4.1963E-04,  5.4490E-02,  -1.8695E-03, 8.9868E-04,

            3.4941E-02,  2.6112E-03,  7.6463E-02,  2.8346E-03,  2.6478E-04,
            -1.3297E-04, -1.8695E-03, 5.2819E-02,  1.0990E-02,

            -3.5179E-02, -4.2663E-02, -3.5302E-03, -5.6482E-04, -1.0781E-03,
            4.1190E-05,  8.9868E-04,  1.0990E-02,  0.1291};

        // copying PP0 onto Matrix PP_INIT
        Matrix PP_INIT(9, 9);
        for (int p = 0; p < 9; p++) {
            for (int q = 0; q < 9; q++) {
                PP_INIT.assign_loc(p, q, PP0[p][q]);
            }
        }
        // getting square root of covariance matrix
        Matrix APP_INIT = PP_INIT.cholesky();

        // drawing Gaussian 9x1 vector with unit std deviation
        Matrix GAUSS_INIT(9, 1);
        for (int r = 0; r < 9; r++) {
            GAUSS_INIT.assign_loc(r, 0, gauss(0, 1));
        }
        // forming stochastic initial state vector
        Matrix XX_INIT = APP_INIT * GAUSS_INIT;
        XX_INIT *= (1 + frax_algnmnt);

        // forming subvectors for initialization and converting tilt to radians
        ESBI.build_vec3(XX_INIT[0], XX_INIT[1], XX_INIT[2]);
        EVBI.build_vec3(XX_INIT[3], XX_INIT[4], XX_INIT[5]);
        RICI.build_vec3(XX_INIT[6], XX_INIT[7], XX_INIT[8]);
        // tilt converted from milliradians to radians
        RICI *= 0.001;
    }
    //-------------------------------------------------------------------------
    // loading module-variables
    // initializations
    RICI.fill(rici);
    EVBI.fill(evbi);
    ESBI.fill(esbi);
}

void INS::ins_accl()
{
    Matrix EFSPB(efspb);
    Matrix EMISA(emisa);
    Matrix ESCALA(escala);
    Matrix EBIASA(ebiasa);
    Matrix EWALKA(ewalka);
    Matrix FSPB = newton->get_FSPB();

    //-------------------------------------------------------------------------
    // computing accelerometer erros without random walk (done in 'ins()')
    Matrix EAB = ESCALA.diamat_vec() + EMISA.skew_sym();
    EFSPB = EWALKA + EBIASA + EAB * FSPB;
    //-------------------------------------------------------------------------
    EFSPB.fill(efspb);
}

void INS::ins_gyro(double int_step)
{
    // local module-variables
    Matrix EUG(eug);
    Matrix EWG(ewg);
    Matrix EWBIB(ewbib);
    // localizing module-variables
    // input data
    Matrix EWALKG(ewalkg);
    Matrix EUNBG(eunbg);
    Matrix EMISG(emisg);
    Matrix ESCALG(escalg);
    Matrix EBIASG(ebiasg);
    // input from other modules
    Matrix WBIB = euler->get_WBIB();
    Matrix FSPB = newton->get_FSPB();
    //-------------------------------------------------------------------------
    // computing cluster misalignment error
    Matrix EGB = ESCALG.diamat_vec() + EMISG.skew_sym();
    Matrix EMISCG = EGB * WBIB;
    Matrix EMSBG = EBIASG + EMISCG;

    // computing gyro spin axis sensitivity (mass unbalance)
    EUG[0] = EUNBG[0] * FSPB[0];
    EUG[1] = EUNBG[1] * FSPB[1];
    EUG[2] = EUNBG[2] * FSPB[2];

    // computing random walk error
    EWG = EWALKG * (1. / sqrt(int_step));

    // combining all uncertainties
    EWBIB = EMSBG + EUG + EWG;

    // gyro measured body rates

    //-------------------------------------------------------------------------
    // loading module-variables
    // diagnostics
    EUG.fill(eug);
    EWG.fill(ewg);
    EWBIB.fill(ewbib);
}

void INS::ins_grav()
{
    // local variable
    Matrix EGRAVI(egravi);
    Matrix ESBI(esbi);
    Matrix SBIIC(sbiic);
    //-------------------------------------------------------------------------
    double dbi = newton->get_dbi();
    double dbic = SBIIC.absolute();
    double ed = dbic - dbi;
    double dum = GM / pow(dbic, 3);
    if (dbic != 0) {
        EGRAVI = ESBI * (-dum) - SBIIC * (3 * ed * dum / dbic);
    }

    EGRAVI.fill(egravi);
}

///////////////////////////////////////////////////////////////////////////////
// INS module
// Member function of class 'Hyper'
// Error equations based on Zipfel, Figure 10.27
// space stabilized INS with GPS and star tracker updates
// when 'mstar'=3 star tracker update received
// when 'mgps'=3 GPS update received
//
// mins = 0 ideal INS (no errors)
//      = 1 space stabilized INS
//
// 030604 Created by Peter H Zipfel
// 040130 GPS update inserted, PZi
// 040212 Star tracker update inserted, PZi
// 050308 Added work-around roll angle singularity, PZi
// 050623 Roll feedback variant for inverted flight, PZi
// 091130 Euler angle clean-up, PZi
///////////////////////////////////////////////////////////////////////////////
void INS::update(double int_step){
    // local variables
    double lonc(0), latc(0);
    double phipc(0);
    double psivdc(0), thtvdc(0);
    double psibdc(0), thtbdc(0), phibdc(0);
    double cthtbd(0);

    // local module-variables
    Matrix VBECB(3, 1);
    Matrix VBECD(3, 1);
    // Matrix EFSPB(3,1);

    Matrix TBIC(3, 3);
    Matrix TDCI(3, 3);

    Matrix WBICI(wbici);
    // Matrix WBICB(3,1);
    // localizing module-variables
    // input data
    // initialization
    Matrix VBIIC(vbiic);
    Matrix SBIIC(sbiic);
    // input from other modules
    double time = get_rettime();
    Matrix GRAVG = environment->get_GRAVG();
    Matrix TBI = kinematics->get_TBI();
    Matrix WBIB = euler->get_WBIB();
    Matrix WBII = euler->get_WBII();
    Matrix SBII = newton->get_IPos();
    Matrix VBII = newton->get_IVel();
    Matrix FSPB = newton->get_FSPB();

    int mroll = 0; // Ambiguous

    //int mgps = hyper[700].integer();

    Matrix SXH(gpsr->position_state);
    Matrix VXH(gpsr->velocity_state);

    //int mstar = hyper[800].integer();
    //Matrix URIC = hyper[830].vec();

    Matrix EWBIB(ewbib);
    Matrix FSPCB(fspcb);
    Matrix EGRAVI(egravi);
    Matrix EUG(eug);
    Matrix EWG(ewg);
    Matrix EFSPB(efspb);
    Matrix WBICB(wbicb);
    // state variables
    Matrix RICID(ricid);
    Matrix RICI(rici);
    Matrix EVBID(evbid);
    Matrix EVBI(evbi);
    Matrix ESBID(esbid);
    Matrix ESBI(esbi);
    // int fsw_count=hyper[577].integer();
    // int fsw_time_flag=hyper[578].integer();
    //-------------------------------------------------------------------------
    // ideal INS output
    if (mins == 0) {
        TBIC = TBI;
        FSPCB = FSPB;
        WBICI = WBII;
        WBICB = WBIB;
        SBIIC = SBII;
        dbic = SBIIC.absolute();
        VBIIC = VBII;
    } else {
        // computing INS derived postion of hyper B wrt center of Earth I
        SBIIC = ESBI + SBII;
        dbic = SBIIC.absolute();

        // calculating attitude errors
        // EWBIB=ins_gyro(WBICB, int_step);
        WBICB = WBIB + EWBIB;
        Matrix RICID_NEW = ~TBI * EWBIB;
        RICI = integrate(RICID_NEW, RICID, RICI, int_step);
        RICID = RICID_NEW;

        // updating tilt with star tracker
        //if (mstar == 3) {
            //RICI = RICI - URIC;
            //// returning flag to star tracker that update was completed
            //mstar = 2;
        //}
        // computed transformation matrix
        Matrix UNI(3, 3);
        UNI.identity();
        Matrix TIIC = UNI - RICI.skew_sym();
        TBIC = TBI * TIIC;

        // calculating velocity error
        // accelerometer error (bias,scale factor,misalignment)
        // EFSPB=ins_accl();
        // acceleration measurement with random walk effect
        FSPCB = EFSPB + FSPB;
        // gravitational error
        // Matrix EGRAVI=ins_grav(ESBI,SBIIC);
        // integrating velocity error equation
        Matrix TICB = ~TBIC;
        Matrix EVBID_NEW =
            TICB * EFSPB - RICI.skew_sym() * TICB * FSPCB + EGRAVI;
        EVBI = integrate(EVBID_NEW, EVBID, EVBI, int_step);
        EVBID = EVBID_NEW;

        // calculating position error
        Matrix ESBID_NEW = EVBI;
        ESBI = integrate(ESBID_NEW, ESBID, ESBI, int_step);
        ESBID = ESBID_NEW;

        // GPS update
        if (gpsr->gps_update) {
            // updating INS navigation output
            SBIIC = SBIIC - SXH;
            VBIIC = VBIIC - VXH;
            // resetting INS error states
            ESBI = ESBI - SXH;
            EVBI = EVBI - VXH;
            // returning flag to GPS that update was completed
            gpsr->gps_update--;
        }
        // computing INS derived position of hyper B wrt center of Earth I
        SBIIC = ESBI + SBII;
        // computing INS derived velocity of hyper B wrt inertial frame I
        VBIIC = EVBI + VBII;
        // computing INS derived body rates in inertial coordinates
        WBICI = ~TBIC * WBICB;

        // diagnostics
        ins_pos_err = ESBI.absolute();
        ins_vel_err = EVBI.absolute();
        ins_tilt_err = RICI.absolute();
    }
    // computing geographic velocity in body coordinates from INS
    Matrix VEIC(3, 1);
    VEIC[0] = -WEII3 * SBIIC[1];
    VEIC[1] = WEII3 * SBIIC[0];
    VEIC[2] = 0;
    Matrix VBEIC = VBIIC - VEIC;
    VBECB = TBIC * VBEIC;
    dvbec = VBECB.absolute();

    // decomposing computed body rates
    ppcx = WBICB[0] * DEG;
    qqcx = WBICB[1] * DEG;
    rrcx = WBICB[2] * DEG;

    // computing indidence angles from INS
    double alphac = atan2(VBECB[2], VBECB[0]);
    double betac = asin(VBECB[1] / dvbec);
    alphacx = alphac * DEG;
    betacx = betac * DEG;

    // incidence angles in load factor plane (aeroballistic)
    double dum = VBECB[0] / dvbec;
    if (fabs(dum) > 1)
        dum = 1 * sign(dum);
    double alppc = acos(dum);

    if (VBECB[1] == 0 && VBECB[2] == 0)
        phipc = 0.;
    // note: if vbeb2 is <EPS the value if phipc is forced to be 0 or PI
    //       to prevent oscillations
    else if (fabs(VBECB[1]) < EPS)
        if (VBECB[2] > 0)
            phipc = 0;
    if (VBECB[2] < 0)
        phipc = PI;
    else
        phipc = atan2(VBECB[1], VBECB[2]);
    alppcx = alppc * DEG;
    phipcx = phipc * DEG;

    // getting long,lat,alt from INS
    cad_geo84_in(lonc, latc, altc, SBIIC, time);

    // getting T.M. of geodetic wrt inertial coord
    TDCI = cad_tdi84(lonc, latc, altc, time);
    loncx = lonc * DEG;
    latcx = latc * DEG;

    // computing geodetic velocity from INS
    VBECD = TDCI * VBEIC;

    // computing flight path angles
    if (VBECD[0] == 0 && VBECD[1] == 0) {
        psivdc = 0;
        thtvdc = 0;
    } else {
        psivdc = atan2(VBECD[1], VBECD[0]);
        thtvdc =
            atan2(-VBECD[2], sqrt(VBECD[0] * VBECD[0] + VBECD[1] * VBECD[1]));
    }
    psivdcx = psivdc * DEG;
    thtvdcx = thtvdc * DEG;

    // computing Euler angles from INS
    Matrix TBD = TBIC * ~TDCI;
    double tbd13 = TBD.get_loc(0, 2);
    double tbd11 = TBD.get_loc(0, 0);
    double tbd33 = TBD.get_loc(2, 2);
    double tbd12 = TBD.get_loc(0, 1);
    double tbd23 = TBD.get_loc(1, 2);

    //*geodetic Euler angles
    // computed pitch angle: 'thtbdc'
    // note: when |tbd13| >= 1, thtbdc = +- pi/2, but cos(thtbdc) is
    // forced to be a small positive number to prevent division by zero
    if (fabs(tbd13) < 1) {
        thtbdc = asin(-tbd13);
        cthtbd = cos(thtbdc);
    } else {
        thtbdc = PI / 2 * sign(-tbd13);
        cthtbd = EPS;
    }
    // computed yaw angle: 'psibdc'
    double cpsi = tbd11 / cthtbd;
    if (fabs(cpsi) > 1)
        cpsi = 1 * sign(cpsi);
    psibdc = acos(cpsi) * sign(tbd12);

    // computed roll angle: 'phibdc'
    double cphi = tbd33 / cthtbd;
    if (fabs(cphi) > 1)
        cphi = 1 * sign(cphi);

    // selecting the Euler roll angle of flight mechanics (not for thtbdc=90 or
    // =-90deg)
    if (mroll == 0 || mroll == 1)
        // roll feedback for right side up
        phibdc = acos(cphi) * sign(tbd23);
    else if (mroll == 2)
        // roll feedback for inverted flight
        phibdc = acos(-cphi) * sign(-tbd23);

    psibdcx = DEG * psibdc;
    thtbdcx = DEG * thtbdc;
    phibdcx = DEG * phibdc;
    //-------------------------------------------------------------------------
    // loading module-variables
    // state variables
    RICID.fill(ricid);
    RICI.fill(rici);
    EVBID.fill(evbid);
    EVBI.fill(evbi);
    ESBID.fill(esbid);
    ESBI.fill(esbi);
    // output to other modules;
    VBIIC.fill(vbiic);
    SBIIC.fill(sbiic);
    WBICI.fill(wbici);
    WBICB.fill(wbicb);
    TBIC.fill(tbic);
    VBECD.fill(vbecd);
    TDCI.fill(tdci);
    FSPCB.fill(fspcb);

    //hyper[700].gets(mgps);

    //hyper[800].gets(mstar);
    // diagnostics
    EWBIB.fill(ewbib);
    EFSPB.fill(efspb);
    EGRAVI.fill(egravi);
}

double INS::get_dvbec() { return dvbec; }
double INS::get_qqcx() { return qqcx; }
double INS::get_rrcx() { return rrcx; }
double INS::get_ppcx() { return ppcx; }
double INS::get_alphacx() { return alphacx; }
double INS::get_betacx() { return betacx; }
double INS::get_phibdcx() { return phibdcx; }
double INS::get_thtbdcx() { return thtbdcx; }
double INS::get_psibdcx() { return psibdcx; }

Matrix INS::get_FSPCB() {
    Matrix FSPCB(3, 1);
    FSPCB.build_vec3(fspcb);
    return FSPCB;
}
Matrix INS::get_SBIIC() {
    Matrix SBIIC(3, 1);
    SBIIC.build_vec3(sbiic);
    return SBIIC;
}
Matrix INS::get_VBIIC() {
    Matrix VBIIC(3, 1);
    VBIIC.build_vec3(vbiic);
    return VBIIC;
}
Matrix INS::get_WBICI() {
    Matrix WBICI(3, 1);
    WBICI.build_vec3(wbici);
    return WBICI;
}
Matrix INS::get_EFSPB() {
    Matrix EFSPB(3, 1);
    EFSPB.build_vec3(efspb);
    return EFSPB;
}
Matrix INS::get_EWALKA() {
    Matrix EWALKA(3, 1);
    EWALKA.build_vec3(ewalka);
    return EWALKA;
}
Matrix INS::get_EMISA() {
    Matrix EMISA(3, 1);
    EMISA.build_vec3(emisa);
    return EMISA;
}
Matrix INS::get_ESCALA() {
    Matrix ESCALA(3, 1);
    ESCALA.build_vec3(escala);
    return ESCALA;
}
Matrix INS::get_EBIASA() {
    Matrix EBIASA(3, 1);
    EBIASA.build_vec3(ebiasa);
    return EBIASA;
}
Matrix INS::get_EUG() {
    Matrix EUG(3, 1);
    EUG.build_vec3(eug);
    return EUG;
}
Matrix INS::get_EWG() {
    Matrix EWG(3, 1);
    EWG.build_vec3(ewg);
    return EWG;
}
Matrix INS::get_EWBIB() {
    Matrix EWBIB(3, 1);
    EWBIB.build_vec3(ewbib);
    return EWBIB;
}
Matrix INS::get_EWALKG() {
    Matrix EWALKG(3, 1);
    EWALKG.build_vec3(ewalkg);
    return EWALKG;
}
Matrix INS::get_EUNBG() {
    Matrix EUNBG(3, 1);
    EUNBG.build_vec3(eunbg);
    return EUNBG;
}
Matrix INS::get_EMISG() {
    Matrix EMISG(3, 1);
    EMISG.build_vec3(emisg);
    return EMISG;
}
Matrix INS::get_ESCALG() {
    Matrix ESCALG(3, 1);
    ESCALG.build_vec3(escalg);
    return ESCALG;
}
Matrix INS::get_EBIASG() {
    Matrix EBIASG(3, 1);
    EBIASG.build_vec3(ebiasg);
    return EBIASG;
}
Matrix INS::get_EGRAVI() {
    Matrix EGRAVI(3, 1);
    EGRAVI.build_vec3(egravi);
    return EGRAVI;
}

Matrix INS::get_TBIC() {
    Matrix TBIC(3, 3);
    TBIC.build_mat33(tbic);
    return TBIC;
}

void INS::set_FSPCB(double n0, double n1, double n2) {
    fspcb[0] = n0;
    fspcb[1] = n1;
    fspcb[2] = n2;
}
void INS::set_SBIIC(double n0, double n1, double n2) {
    sbiic[0] = n0;
    sbiic[1] = n1;
    sbiic[2] = n2;
}
void INS::set_VBIIC(double n0, double n1, double n2) {
    vbiic[0] = n0;
    vbiic[1] = n1;
    vbiic[2] = n2;
}
void INS::set_WBICI(double n0, double n1, double n2) {
    wbici[0] = n0;
    wbici[1] = n1;
    wbici[2] = n2;
}
void INS::set_EFSPB(double n0, double n1, double n2) {
    efspb[0] = n0;
    efspb[1] = n1;
    efspb[2] = n2;
}
void INS::set_EWALKA(double n0, double n1, double n2) {
    ewalka[0] = n0;
    ewalka[1] = n1;
    ewalka[2] = n2;
}
void INS::set_EMISA(double n0, double n1, double n2) {
    emisa[0] = n0;
    emisa[1] = n1;
    emisa[2] = n2;
}
void INS::set_ESCALA(double n0, double n1, double n2) {
    escala[0] = n0;
    escala[1] = n1;
    escala[2] = n2;
}
void INS::set_EBIASA(double n0, double n1, double n2) {
    ebiasa[0] = n0;
    ebiasa[1] = n1;
    ebiasa[2] = n2;
}
void INS::set_EUG(double n0, double n1, double n2) {
    eug[0] = n0;
    eug[1] = n1;
    eug[2] = n2;
}
void INS::set_EWG(double n0, double n1, double n2) {
    ewg[0] = n0;
    ewg[1] = n1;
    ewg[2] = n2;
}
void INS::set_EWBIB(double n0, double n1, double n2) {
    ewbib[0] = n0;
    ewbib[1] = n1;
    ewbib[2] = n2;
}
void INS::set_EWALKG(double n0, double n1, double n2) {
    ewalkg[0] = n0;
    ewalkg[1] = n1;
    ewalkg[2] = n2;
}
void INS::set_EUNBG(double n0, double n1, double n2) {
    eunbg[0] = n0;
    eunbg[1] = n1;
    eunbg[2] = n2;
}
void INS::set_EMISG(double n0, double n1, double n2) {
    emisg[0] = n0;
    emisg[1] = n1;
    emisg[2] = n2;
}
void INS::set_ESCALG(double n0, double n1, double n2) {
    escalg[0] = n0;
    escalg[1] = n1;
    escalg[2] = n2;
}
void INS::set_EBIASG(double n0, double n1, double n2) {
    ebiasg[0] = n0;
    ebiasg[1] = n1;
    ebiasg[2] = n2;
}
void INS::set_EGRAVI(double n0, double n1, double n2) {
    egravi[0] = n0;
    egravi[1] = n1;
    egravi[2] = n2;
}
