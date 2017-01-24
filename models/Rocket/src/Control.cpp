#include "Control.hh"

void Control::default_data(){
}

double  Control::get_delecx() { return delecx; }
double  Control::get_delrcx() { return delrcx; }

void Control::initialize(INS *i, Newton *ntn, Environment *env, Propulsion *plp, AeroDynamics *aero){
    ins = i;
    environment = env;
    newton = ntn;
    propulsion = plp;
    aerodynamics = aero;
}

///////////////////////////////////////////////////////////////////////////////
//'control' module
// Member function of class 'Control'
//
// maut = |mauty|mautp|
//
//         mauty = 0 no control, fixed control surfaces
//               = 5 yaw acceleration control for yaw-to-turn
//
//               mautp = 0 no control, fixed control surfaces
//                     = 3 pitch acceleration control
//
// 030520 Created by Peter H Zipfel
// 091214 Modified for ROCKET6, PZi
///////////////////////////////////////////////////////////////////////////////

void Control::control(double int_step){
    // localizing module-variables
    // input from other modules
    int mprop = propulsion->mprop;
    double gymax = aerodynamics->get_gymax();
    //-------------------------------------------------------------------------
    // decoding control flag
    // std::cout<<"Called : Control"<<'\n';
    int mauty = maut / 10;
    int mautp = (maut % 10);
    // fsw_time_flag++;
    // if(fsw_time_flag == fsw_count)
    //{
    // calling acceleration controller in yaw-plane
    if (mauty == 5) {
        // limiting lateral acceleration by max g capability (estabished by
        // 'betalimx')
        if (alcomx > gymax)
            alcomx = gymax;
        if (alcomx < -gymax)
            alcomx = -gymax;
        // for booster while thrusting - TVC control
        if (mprop)
            delrcx = control_yaw_accel(alcomx, int_step);
    }
    // calling acceleration controller in normal-plane
    if (mautp == 3) {
        // if(ancomx>gnmax) ancomx=gnmax;
        // if(ancomx<-gnmax) ancomx=-gnmax;
        // for booster while thrusting - TVC control
        if (mprop)
            delecx = control_normal_accel(ancomx, int_step);
    }
    if (mautp == 4) {
        if (mprop)
            delecx = control_pitch_rate(qqdx);
    }
    // limiting control commands
    if (fabs(delecx) > delimx)
        delecx = delimx * sign(delecx);
    if (fabs(delrcx) > drlimx)
        delrcx = drlimx * sign(delrcx);

    // diagnostic output
    alcomx_actual = alcomx;
    ancomx_actual = ancomx;
    //-------------------------------------------------------------------------
}

///////////////////////////////////////////////////////////////////////////////
// Acceleration controller in normal (pitch) plane
// Member function of class 'Control'
// Employs pole placement technique (no matrix inversion required)
// Ref: Zipfel, p.416
// Feedback signals are: body rate (gyro) and acceleration (accel)
//
// (1) Calculates two feedback and one feed-forward gains
//     based on input of dominant closed loop conjugate complex
//     roots
// (2) Calculates the commanded pitch control deflection
//
// Return output
//        delecx = pitch control command - deg
// Parameter input
//        ancomx = normal loadfactor command - g
//        int_step = integration step size - s
//
// 021015 Created by Peter H Zipfel
// 060120 Added variable bandwidth, PZi
// 091214 Modified for ROCKET6, PZi
///////////////////////////////////////////////////////////////////////////////

double Control::control_normal_accel(double ancomx, double int_step){
    Matrix GAINFP(3, 1);
    double gainfb1(0);
    double gainfb2(0);
    double gainfb3(0);

    double dyb = aerodynamics->get_dyb();
    double dnb = aerodynamics->get_dnb();
    double dnr = aerodynamics->get_dnr();
    double dndr = aerodynamics->get_dndr();

    // input from other modules
    double pdynmc = environment->get_pdynmc();
    double dla = aerodynamics->get_dla();
    double dma = aerodynamics->get_dma();
    double dmq = aerodynamics->get_dmq();
    double dmde = aerodynamics->get_dmde();
    double dvbec = ins->dvbec;
    double qqcx = ins->qqcx;
    Matrix FSPCB(ins->fspcb);
    //-------------------------------------------------------------------------
    // calculating online close loop poles
    waclp = (0.1 + 0.5e-5 * (pdynmc - 20e3)) * (1 + factwaclp);
    paclp = 0.7 + 1e-5 * (pdynmc - 20e3) * (1 + factwaclp);

    // calculating three feedback gains

    gainfb3 = waclp * waclp * paclp / (dla * dmde);
    gainfb2 = (2 * zaclp * waclp + paclp + dmq - dla / dvbec) / dmde;
    gainfb1 = (waclp * waclp + 2. * zaclp * waclp * paclp + dma +
               dmq * dla / dvbec - gainfb2 * dmde * dla / dvbec) /
                  (dla * dmde) -
              gainp;

    // gainfb3=zaclp*zaclp*waclp*waclp*paclp/(dmde*dla);
    // gainfb2=(2.*zaclp*waclp+paclp+dmq-dla/dvbec)/dmde;
    // gainfb1=(zaclp*zaclp*waclp*waclp+2.*zaclp*waclp*paclp+dma+dmq*dla/dvbec-gainfb2*dmde*dla/dvbec)-gainp;


    // pitch loop acceleration control, pitch control command
    double fspb3 = FSPCB[2];
    double zzd_new = AGRAV * ancomx + fspb3;
    zz = integrate(zzd_new, zzd, zz, int_step);
    zzd = zzd_new;
    double dqc =
        -gainfb1 * (-fspb3) - gainfb2 * qqcx * RAD + gainfb3 * zz + gainp * zzd;
    double delecx = dqc * DEG;

    // diagnostic output
    GAINFP.build_vec3(gainfb1, gainfb2, gainfb3);
    //--------------------------------------------------------------------------
    // loading module-variables
    GAINFP.fill(gainfp);

    return delecx;
}
///////////////////////////////////////////////////////////////////////////////
// Acceleration controller in lateral (yaw) plane
// Member function of class 'Control'
// Employs pole placement technique (no matrix inversion required)
// Ref: Zipfel, p.416
// Feedback signals are: body rate (gyro) and acceleration (accel)
//
// (1) Calculates two feedback and one feed-forward gains
//     based on input of dominant closed loop conjugate complex
//     roots
// (2) Calculates the commanded yaw control deflection
//
// Return output
//        drcx = yaw control command - deg
// Parameter input
//        alcomx = lateral loadfactor command = g
//        int_step = integration step size - s
//
// 050104 Adopted from DRM sim, PZi
// 060120 Added variable bandwidth, PZi
// 091214 Modified for ROCKET6, PZi
///////////////////////////////////////////////////////////////////////////////

double Control::control_yaw_accel(double alcomx, double int_step){
    // local module-variables
    Matrix GAINFY(3, 1);

    // input from other modules
    double pdynmc = environment->get_pdynmc();
    double dyb = aerodynamics->get_dyb();
    double dnb = aerodynamics->get_dnb();
    double dnr = aerodynamics->get_dnr();
    double dndr = aerodynamics->get_dndr();
    double dvbe = newton->get_dvbe();
    double rrcx = ins->rrcx;
    Matrix FSPCB(ins->fspcb);

    //-------------------------------------------------------------------------
    // calculating close loop poles
    wacly = (0.1 + 0.5e-5 * (pdynmc - 20e3)) * (1 + factwacly);
    pacly = 0.7 + 1e-5 * (pdynmc - 20e3) * (1 + factwacly);

    // calculating three feedback gains
    double gainfb3 = -wacly * wacly * pacly / (dyb * dndr);
    double gainfb2 = (2 * zacly * wacly + pacly + dnr + dyb / dvbe) / dndr;
    double gainfb1 = (-wacly * wacly - 2. * zacly * wacly * pacly + dnb +
                      dnr * dyb / dvbe - gainfb2 * dndr * dnb / dvbe) /
                         (dyb * dndr) -
                     gainy;

    // yaw loop acceleration controller, yaw control command
    double fspb2 = FSPCB.get_loc(1, 0);
    double yyd_new = AGRAV * alcomx - fspb2;
    yy = integrate(yyd_new, yyd, yy, int_step);
    yyd = yyd_new;
    double drc =
        -gainfb1 * fspb2 - gainfb2 * rrcx * RAD + gainfb3 * yy + gainy * yyd;
    double drcx = drc * DEG;

    // diagnostic output
    GAINFY.build_vec3(gainfb1, gainfb2, gainfb3);
    //--------------------------------------------------------------------------
    GAINFY.fill(gainfy);

    return drcx;
}
//--------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////
// Pitch rate controller in pitch plane
// Member function of class 'Control'
// Employs pole placement technique (no matrix inversion required)
// Ref: Zipfel, p.416
// Feedback signals are: body rate (gyro) and acceleration (accel)s
//
//
// 20160427 Modified for ROCKET6, by Lai
///////////////////////////////////////////////////////////////////////////////
double Control::control_pitch_rate(double qqdx){
    double zrate(0);
    double wnlagr(0);
    double dqcx(0);

    // input from aerodynamic
    double pdynmc = environment->get_pdynmc();
    double dla = aerodynamics->get_dla();
    double dma = aerodynamics->get_dma();
    double dmq = aerodynamics->get_dmq();
    double dmde = aerodynamics->get_dmde();
    double dvbec = ins->dvbec;
    double dnd = aerodynamics->get_dnd();
    double qqcx = ins->qqcx;

    //-------------------------------------------------------------------------
    // parameters of open loop angular rate transfer function
    zrate = dla / dvbec - dma * dnd / (dvbec * dmde);
    double aa = dla / dvbec - dmq;
    double bb = -dma - dmq * dla / dvbec;

    // feecback gain of rate loop, given desired close loop 'zetlager'
    double dum1 = (aa - 2. * zaclp * zaclp * zrate);
    double dum2 = aa * aa - 4. * zaclp * zaclp * bb;
    double radix = dum1 * dum1 - dum2;

    if (radix < 0.)
        radix = 0.;
    grate = (-dum1 + sqrt(radix)) / (dmde);

    // natural frequency of closed rate loop
    double dum3 = grate * dmde * zrate;
    radix = bb + dum3;
    if (radix < 0.)
        radix = 0.;
    wnlagr = sqrt(radix);
    double waclp = wnlagr;

    dqcx = qqdx - DEG * grate * qqcx;
    double delecx = dqcx;

    return delecx;
}
