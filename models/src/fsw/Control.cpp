#include "fsw/Control.hh"

#include "math/integrate.hh"
#include "math/utility.hh"
#include "aux/global_constants.hh"

Control::Control()
    :   VECTOR_INIT(GAINFP, 3),
        VECTOR_INIT(GAINFY, 3),
        MATRIX_INIT(GAINGAM, 3, 1)
{
    this->default_data();
}

Control::Control(const Control& other)
    :   VECTOR_INIT(GAINFP, 3),
        VECTOR_INIT(GAINFY, 3),
        MATRIX_INIT(GAINGAM, 3, 1)
{
    this->default_data();
}

Control& Control::operator=(const Control& other){
    if(&other == this)
        return *this;

    return *this;
}

void Control::default_data(){
}

double  Control::get_delecx() { return delecx; }
double  Control::get_delrcx() { return delrcx; }

void Control::initialize(){
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
    int mprop = grab_mprop();
    double gymax = grab_gymax();
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
    if (mautp == 5){
        if(mprop)
            delecx = control_gamma(thtvdcomx);
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
    double gainfb1(0);
    double gainfb2(0);
    double gainfb3(0);

    double dyb = grab_dyb();
    double dnb = grab_dnb();
    double dnr = grab_dnr();
    double dndr = grab_dndr();

    // input from other modules
    double pdynmc = grab_pdynmc();
    double dla = grab_dla();
    double dma = grab_dma();
    double dmq = grab_dmq();
    double dmde = grab_dmde();
    double dvbec = grab_dvbec();
    double qqcx = grab_qqcx();
    arma::vec3 FSPCB = grab_FSPCB();
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
    GAINFP = arma::vec3({gainfb1, gainfb2, gainfb3});
    //--------------------------------------------------------------------------
    // loading module-variables

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
    // input from other modules
    double pdynmc = grab_pdynmc();
    double dyb = grab_dyb();
    double dnb = grab_dnb();
    double dnr = grab_dnr();
    double dndr = grab_dndr();
    double dvbe = grab_dvbe();
    double rrcx = grab_rrcx();
    arma::vec3 FSPCB = grab_FSPCB();

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
    double fspb2 = FSPCB(1, 0);
    double yyd_new = AGRAV * alcomx - fspb2;
    yy = integrate(yyd_new, yyd, yy, int_step);
    yyd = yyd_new;
    double drc =
        -gainfb1 * fspb2 - gainfb2 * rrcx * RAD + gainfb3 * yy + gainy * yyd;
    double drcx = drc * DEG;

    // diagnostic output
    GAINFY = arma::vec3({gainfb1, gainfb2, gainfb3});
    //--------------------------------------------------------------------------

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
    double pdynmc = grab_pdynmc();
    double dla = grab_dla();
    double dma = grab_dma();
    double dmq = grab_dmq();
    double dmde = grab_dmde();
    double dvbec = grab_dvbec();
    double dnd = grab_dnd();
    double qqcx = grab_qqcx();

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


double Control::control_gamma(double thtvdcomx)
{
    //local variables
    arma::mat33 AA;
    arma::vec3 BB;
    arma::mat33 DP;
    arma::vec3 DD;
    arma::vec3 HH;

    //local module-variables
    // arma::mat GAINGAM(3,3);
    // GAINGAM.zeros();
    double gainff=0;

    //localizing module-variables
    //input data
    double pgam = this->pgam;
    double wgam = this->wgam;
    double zgam = this->zgam;
    //input from other modules
    double dvbe = grab_dvbe();
    double dla = grab_dla();
    double dlde = grab_dlde();
    double dma = grab_dma();
    double dmq = grab_dmq();
    double dmde = grab_dmde();
    double qqcx = grab_qqcx();
    double dvbec = grab_dvbec();
    double thtvdcx = grab_thtvdcx();
    double thtbdcx = grab_thtbdcx();
    //--------------------------------------------------------------------------

    //prevent division by zero
    if(dvbec==0)dvbec=dvbe;

    //building fundamental matrices (body rate, acceleration, fin deflection)
    AA(0,0) = dmq;
    AA(0,1) = dma;
    AA(0,2) = -dma;
    AA(1,0) = 1.;
    AA(1,1) = 0.0;
    AA(1,2) = 0.0;
    AA(2,0) = 0.0;
    AA(2,1) = dla/dvbec;
    AA(2,2) = -dla/dvbec;
    //AA.build_mat33(dmq,dma,-dma,1.,0.,0.,0.,dla/dvbec,-dla/dvbec);
    BB(0) = dmde;
    BB(1) = 0.0;
    BB(2) = dlde/dvbec;
    //BB.build_vec3(dmde,0.,dlde/dvbec);

    //feedback gains from closed-loop pole placement
    double am=2.*zgam*wgam+pgam;
    double bm=wgam*wgam+2.*zgam*wgam*pgam;
    double cm=wgam*wgam*pgam;
    // double v11=dmde;
    // double v12=0.;
    // double v13=dlde/dvbec;
    // double v21=dmde*dla/dvbec-dlde*dma/dvbec;
    // double v22=dmde;
    // double v23=-dmq*dlde/dvbec;
    // double v31=0.;
    // double v32=v21;
    // double v33=v21;

    DP(0,0) = dmde;
    DP(0,1) = 0.0;
    DP(0,2) = dlde/dvbec;
    DP(1,0) = dmde*dla/dvbec-dlde*dma/dvbec;
    DP(1,1) = dmde;
    DP(1,2) = -dmq*dlde/dvbec;
    DP(2,0) = 0.0;
    DP(2,1) = dmde*dla/dvbec-dlde*dma/dvbec;
    DP(2,2) = dmde*dla/dvbec-dlde*dma/dvbec;
    // DP.build_mat33(v11,v12,v13,v21,v22,v23,v31,v32,v33);

    DD(0) = am+dmq-dla/dvbec;
    DD(1) = bm+dma+dmq*dla/dvbec;
    DD(2) = cm;
    // DD.build_vec3(am+dmq-dla/dvbec,bm+dma+dmq*dla/dvbec,cm);
    arma::mat33 DPI=inv(DP);
    GAINGAM=DPI*DD;


    //steady-state feed-forward gain to achieve unit gamma response
    arma::mat33 DUM33=AA-BB*trans(GAINGAM);
    arma::mat33 IDUM33=inv(DUM33);
    arma::vec DUM3=IDUM33*BB;

    HH(0) = 0.0;
    HH(1) = 0.0;
    HH(2) = 1.;
    // HH.build_vec3(0.,0.,1.);
    double denom=dot(HH,DUM3);
    gainff=-1./denom;

    //pitch control command
    double thtc=gainff*thtvdcomx*RAD;
    double qqf=GAINGAM(0,0)*qqcx*RAD;
    double thtbgf=GAINGAM(1,0)*thtbdcx*RAD;
    double thtugf=GAINGAM(2,0)*thtvdcx*RAD;
    double delec=thtc-(qqf+thtbgf+thtugf);
    double delecx=delec*DEG;

    //--------------------------------------------------------------------------
    //loading module-variables
    //diagnostics
    // hyper[566].gets_vec(GAINGAM);
    // hyper[567].gets(gainff);

    return delecx;
}


void Control::set_thtvdcomx(double in) { this->thtvdcomx = in; }
void Control::set_maut(double in) { this->maut = in; }
void Control::set_delimx(double in) { this->delimx = in; }
void Control::set_drlimx(double in) { this->drlimx = in; }
void Control::set_pgam(double in) { this->pgam = in; }
void Control::set_wgam(double in) { this->wgam = in; }
void Control::set_zgam(double in) { this->zgam = in; }