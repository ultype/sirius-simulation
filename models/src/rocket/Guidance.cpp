#include "rocket/Guidance.hh"

#include "sim_services/include/simtime.h"

void Guidance::default_data(){
    init_flag = 1;
    inisw_flag = 1;
    skip_flag = 1;
    ipas_flag = 1;
    ipas2_flag = 1;
    print_flag = 1;
}

void Guidance::initialize(INS *i, Newton *ntn, Propulsion *plp){
    ins = i;
    newton = ntn;
    propulsion = plp;
}

///////////////////////////////////////////////////////////////////////////////
// Guidance module
// Member function of class 'Guidance'
//
//       mguide =  0 no guidance
//              =  5 linear tangent guidance law (LTG) for rocket ascent
//
// 030616 Created by Peter H Zipfel
// 040330 Added LTG guidance, PZi
// 091214 Modified for ROCKET6, PZi
//////////////////////////////////////////////////////////////////////////////

void Guidance::guidance(double int_step)
{
    // local module variable
    Matrix UTIC(3, 1);

    int mprop = propulsion->get_mprop();
    Matrix UTBC(utbc);
    Matrix TBIC = ins->get_TBIC();
    //-------------------------------------------------------------------------
    // zeroing UTBC,if no guidance
    if (mguide == 0)
        UTBC.zero();

    // LTG guidance, starting LTG clock
    if (mguide == 5) {
        // start LTG clock
        if (init_flag) {
            init_flag = 0;
            time_ltg = 0;
        } else {
            time_ltg += int_step;
        }
        // calling LTG guidance after every ltg_step
        if (time_ltg > ltg_step * ltg_count) {
            // std::cout<<"Called : guidance_ltg"<<'\n';
            ltg_count++;
            UTIC = guidance_ltg(mprop, int_step, time_ltg);
            UTBC = TBIC * UTIC;
        }
    }
    //-------------------------------------------------------------------------
    propulsion->set_thrust((Propulsion::THRUST_TYPE)mprop);
    UTBC.fill(utbc);
    // diagnostics
    UTIC.fill(utic);
}
///////////////////////////////////////////////////////////////////////////////
// Linear tangent guidance (LTG) for ascent
// mguide=5
//
// Ref: Jaggers, R.F."Multistage Linear Tangent Ascent Guidance
//     as Baselined for the Space Shuttle Vehicle", NASA MSC,
//     Internal Note MSC-EG-72-39, June 1972
//
// Uses optimal linear tangent guidance law for up to 3 stages
// of constant rocket thrust
//
// Return output:
//        UTIC(3) = unit thrust vector command in inertial coor. - ND
// Parameter input
//        int_step = integration step
//        time_ltg = time elapsed since start of LTG (mguide=1)
// Nomenclature
//        Capitalized variables are 3x1 vectors, unless a capital
//        'N' is appended then the array is used to store information for each
//        of the n stages (max n=3)
//
// 040319 Converted from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Guidance::guidance_ltg(int &mprop, double int_step, double time_ltg)
{
    // local variables
    // Parameter output 'guidance_ltg_crct()'
    Matrix VMISS(3, 1);  // velocity miss - m/s
    // Parameter input 'guidance_ltg_crct()'
    Matrix SPII(3, 1);  // predicted inertial position vector - m
    Matrix VPII(3, 1);  // predicted inertial velocity vector - m/s
    // Paramter output of 'guidance_ltg_tgo()'
    double tgop(0);        // time-to-go of previous computation cycle - s
    Matrix BURNTN(3, 1);   // burn time duration left in n-th stage
                           // -> used in '_igrl()' - s
    Matrix L_IGRLN(3, 1);  // velocity to-be-gained by n-th stage at 'time_ltg'
                           // -> used in '_igrl()' - m/s
    Matrix TGON(3, 1);     // burn durations remaining, summed over n-th and
                           // previous stages, towards reaching end-state - s
    double l_igrl(0);      // L-integral (velocity to be gained - m/s)
    int nstmax(0);         // maximum # of stages needed , used in '_igrl()'
    // Parameter input/output of 'guidance_ltg_tgo()'
    Matrix TAUN(3, 1);  // characteristic time of stages - s
    // Parameter input of 'guidance_ltg_tgo()'
    Matrix VEXN(3, 1);  // exhaust velocity of n-th stage - m/s
    // burn out time of stages - s
    Matrix BOTN(4, 1);  // burn-out time epoch of n-th stage,
                        // in 'time_ltg' clock time - s
    //(0-th stage is dummy stage with BOTN[0]=0, has dimension BOTN(4,1))
    double vgom(0);   // desired velocity to be gained (absolute value) - m/s
    double amag1(0);  // absolute value of current vehicle acceleration - m/s^2
    // Parameter output 'guidance_ltg_igrl()'
    double s_igrl(0);    // thrust integral, postion - m
    double j_igrl(0);    // thrust integral, velocit*time - m
    double q_igrl(0);    // thrust integral, position*time - m*s
    double h_igrl(0);    // thrust integral, velocity*time^2 - m*s
    double p_igrl(0);    // thrust integral, position*time^2 - m*s^2
    double j_over_l(0);  // j_over_l= time remaining - s
    double tlam(0);      // time of thrust integraton - s
    double qprime(0);    // conditioned q-integral - m*s
    // Parameter output 'guidance_ltg_trate()'
    Matrix ULAM(3, 1);  // unit thrust vector in direction of VGO - ND
    Matrix LAMD(3, 1);  // derivative of unit thrust vector - 1/s

    // local module-variables
    Matrix UTIC(3, 1);

    // restoring saved variables
    Matrix RBIAS(rbias);
    Matrix RGRAV(rgrav);
    Matrix RGO(rgo);
    Matrix VGO(vgo);
    Matrix SDII(sdii);
    Matrix UD(ud);
    Matrix UY(uy);
    Matrix UZ(uz);
    // input from other modules
    double time = get_rettime();
    double dbi = newton->get_dbi();
    double dvbi = newton->get_dvbi();
    double thtvdx = newton->get_thtvdx();
    double fmassr = propulsion->get_fmassr();
    Matrix VBIIC = ins->get_VBIIC();
    Matrix SBIIC = ins->get_SBIIC();
    Matrix TBIC = ins->get_TBIC();
    Matrix FSPCB = ins->get_FSPCB();
    //-------------------------------------------------------------------------
    // preparing necessary variables
    Matrix ABII = ~TBIC * FSPCB;
    amag1 = ABII.absolute();

    // initializing
    if (inisw_flag) {
        inisw_flag = 0;

        // initializing predicted state to current state
        SPII = SBIIC;
        VPII = VBIIC;

        // calling corrector for initialization
        guidance_ltg_crct(SDII, UD, UY, UZ, VMISS,  // output
                          VGO,                      // input-output
                          dbi_desired, dvbi_desired, thtvdx_desired, SPII, VPII,
                          SBIIC, VBIIC);  // input
    } else
        // updating velocity to go
        VGO = VGO - ABII * ltg_step;

    // velocity-to-go magnitude
    vgom = VGO.absolute();

    // building data vector of the stages
    TAUN.build_vec3(char_time1, char_time2, char_time3);
    VEXN.build_vec3(exhaust_vel1, exhaust_vel2, exhaust_vel3);

    // building array of burn-out time epochs of n-th stage, in 'time_ltg' clock
    // time - s
    // burn-out epoch vector starts with dummy o-th stage then stages 1,2,3
    BOTN[0] = 0;
    BOTN[1] = burnout_epoch1;
    BOTN[2] = burnout_epoch2;
    BOTN[3] = burnout_epoch3;

    // calling time-to-go function
    guidance_ltg_tgo(tgop, BURNTN, L_IGRLN, TGON, l_igrl, nstmax,  // output
                     tgo, nst, TAUN,  // input/output
                     VEXN, BOTN, delay_ignition, vgom, amag1, amin, time_ltg,
                     num_stages);  // input
    // calling thrust integral function
    guidance_ltg_igrl(s_igrl, j_igrl, q_igrl, h_igrl, p_igrl, j_over_l, tlam,
                      qprime,  // output
                      nst, nstmax, BURNTN, L_IGRLN, TGON, TAUN, VEXN, l_igrl,
                      time_ltg);  // input
    // calling turning rate function
    guidance_ltg_trate(ULAM, LAMD, RGO,  // output
                       ipas2_flag,       // input-output
                       VGO, s_igrl, q_igrl, j_over_l, lamd_limit, vgom,
                       time_ltg,  // input
                       tgo, tgop, SDII, SBIIC, VBIIC, RBIAS, UD, UY, UZ,
                       RGRAV);  // throughput to '_rtgo(()'

    // calculating thrust command vector in inertial coordinates
    Matrix TC =
        ULAM + LAMD * (time_ltg - tlam);  // same as: TC=ULAM+LAMD*(-j_over_l)

    // calculating output thrust unit vector after skipping 10 'guid_step' delay
    // (settling of transients)
    if (skip_flag) {
        skip_flag++;
        if (skip_flag == 10)
            skip_flag = 0;
    } else
        UTIC = TC.univec3();

    // calling end-state predictor and corrector
    guidance_ltg_pdct(SPII, VPII, RGRAV, RBIAS  // output
                      ,
                      LAMD, ULAM, l_igrl, s_igrl, j_igrl, q_igrl, h_igrl,
                      p_igrl, j_over_l, qprime  // input
                      ,
                      SBIIC, VBIIC, RGO, tgo);
    guidance_ltg_crct(SDII, UD, UY, UZ, VMISS  // output
                      ,
                      VGO  // input-output
                      ,
                      dbi_desired, dvbi_desired, thtvdx_desired, SPII, VPII,
                      SBIIC, VBIIC);  // input

    // motor burning while fuel available
    if (fmassr > 0)
        mprop = 4;

    // cut-off logic
    if (tgo < ltg_step) {
        beco_flag = 1;
        mprop = 0;
    }
    // boost engine cut-off print-out
    double ddb(0);
    double dvdb(0);
    double thtvddbx(0);
    if (beco_flag && print_flag) {
        print_flag = 0;
        ddb = dbi_desired - dbi;
        dpd = 0.0;
        dbd = 0.0;
        dvdb = dvbi_desired - dvbi;
        thtvddbx = thtvdx_desired - thtvdx;
        std::cout << " *** Boost engine cut-off time = " << time
                  << " sec ***\n";
        std::cout << "     Orbital position dbi = " << dbi
                  << " m \tInertial speed dvbi = " << dvbi
                  << " m/s \tFlight path angle thtvdx = " << thtvdx << " deg\n";
        std::cout << "     Position error   ddb = " << ddb
                  << " m \t\tSpeed error    dvdb = " << dvdb
                  << " m/s\tAngle error     thtvddbx = " << thtvddbx
                  << " deg\n";
    }
    //-------------------------------------------------------------------------
    // loading dignostic module-variables
    // output to other modules
    // saving variables
    RBIAS.fill(rbias);
    RGRAV.fill(rgrav);
    RGO.fill(rgo);
    VGO.fill(vgo);
    SDII.fill(sdii);
    UD.fill(ud);
    UY.fill(uy);
    UZ.fill(uz);
    ULAM.fill(ulam);
    LAMD.fill(_lamd);

    // returning unit thrust vector command
    return UTIC;
}

///////////////////////////////////////////////////////////////////////////////
// Time-to-go to desired end-state
//
// Parameter output:
//    tgop = time-to-go of previous computation cycle - s
//    BURNTN(3) = burn time duration left in n-th stage -> used in '_igrl()' - s
//    L_IGRLN(3) = velocity to-be-gained by n-th stage at 'time_ltg'
//                 -> used in '_igrl()' - m/s
//    TGON(3) = burn durations remaining, summed over n-th and previous
//              stages, towards reaching end-state - s
//    l_igrl = L-integral (velocity-to-be-gained by remaining stages - m/s)
//             nstmax = maximum # of stages needed , used in '_igrl()'
//
// Parameter input/output:
//    tgo = time-to-go to desired end-state - s
//    nst = n-th stage number -> used in '_igrl()'
//    TAUN(3) = characteristic time of n-th stage - s
//
// Parameter input:
//    VEXN(3) = exhaust velocity of n-th stage - m/s
//    BOTN(4) = burn-out time epoch of n-th stage, in 'time_ltg' clock time - s
//          (0-th stage is dummy stage with BOTN[0]=0, has dimension BOTN(4,1))
//    delay_ignition = delay of booster ignition after stating (input) -s
//    vgom = desired velocity to be gained (absolute value) - m/s
//    amag1 = absolute value of current vehicle acceleration - m/s^2
//    amin = minimum accleration for TAUN calcualtions (input) - m/s^2
//    time_ltg = time clock started when LTG is engaged -s
//    num_stages = number of stages (initialized in input)
//
// 040322 Converted from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Guidance::guidance_ltg_tgo(double &tgop,
                             Matrix &BURNTN,
                             Matrix &L_IGRLN,
                             Matrix &TGON,
                             double &l_igrl,
                             int &nstmax,
                             double &tgo,
                             int &nst,
                             Matrix &TAUN,
                             Matrix VEXN,
                             Matrix BOTN,
                             double delay_ignition,
                             double vgom,
                             double amag1,
                             double amin,
                             double time_ltg,
                             int num_stages)
{
    // local variables
    int i(0);

    //-------------------------------------------------------------------------
    // initializing to first stage
    if (ipas_flag)
        nst = 1;
    // save previous time-to-go for use in '_rtgo()' ( initially: tgo=0)
    tgop = tgo;

    // initialize for-loop
    tgo = 0;
    l_igrl = 0;
    nstmax = num_stages;

    // advancing stage counter (outside for-loop, stage counter is nst=1,2,3)
    if ((time_ltg >= BOTN[nst]))
        nst++;

    // looping through remaining stages (inside for-loop, stage counter is
    // i=0,1,2)
    for (i = nst - 1; i < nstmax; i++) {
        // if current stage, re-calculating characteristic time
        if (i == (nst - 1)) {
            // first stage: no change (BOTN[0]=0)
            TAUN[nst - 1] = TAUN[nst - 1] - (time_ltg - BOTN[nst - 1]);
        }

        // if sufficient accel.and ignition of stage has occured then
        // re-calculate
        if ((amag1 >= amin) && (time_ltg > (BOTN[nst - 1] + delay_ignition))) {
            // re-calculating 'tau' using current acceleration
            TAUN[nst - 1] = VEXN[nst - 1] * (1 / amag1);
        }

        // calculating remaining boost times
        if (i == (nst - 1))
            // remaining burn time left in current stage
            BURNTN[i] = BOTN[i + 1] - time_ltg;
        else
            // burn time in other stages (assuming 'delay_ignition'=0)
            BURNTN[i] = BOTN[i + 1] - BOTN[i];

        // calculating velocity gained of (n=i+1)-th stage ('L_IGRLN[i]')
        double dum1 = TAUN[i];
        double dum2 = BURNTN[i];
        L_IGRLN[i] = -VEXN[i] * log(1 - dum2 / dum1);

        // accumulating velocity to be gained for the remainder of the boost
        // phase
        l_igrl += L_IGRLN[i];
        // if velocity-to-be-gained is less than desired
        if (l_igrl < vgom) {
            // update tgo for remaining burn times left
            tgo += BURNTN[i];

            // load into array: time to reach end-state for each stage (i=0,1,2)
            TGON[i] = tgo;
        } else {
            i++;
            break;
        }
    }
    // recomputing the velocity-to-be-gained to adjust to vgom
    nstmax = i;  // After an ordinary or 'break' termination of the for-loop,
                 // 'i' is increased by 'one',
    // and is now equal to the number of stages needed to meet the end-state
    // conditons.
    // However, for the following arrays, where 'i' is the offset index,it must
    // be decreased
    // by 'one' to designate the last required stage

    // subtracting from velocity-to-be-gained integral the last stage value
    l_igrl = l_igrl - L_IGRLN[i - 1];

    // re-calculating last stage boost time
    double almx = vgom - l_igrl;
    L_IGRLN[i - 1] = almx;
    double dum3 = VEXN[i - 1];
    BURNTN[i - 1] = TAUN[i - 1] * (1 - exp(-almx / dum3));

    // updating tgo
    tgo += BURNTN[i - 1];  // accumulating burn intervals
                           // remaining until end-state
    TGON[i - 1] = tgo;     // loading burn duration left

    // setting integral 'l_igrl' to desired velocity-to-be-gained
    l_igrl = vgom;

    if (ipas_flag) {
        // initializing 'tgop', to be used in '_rtgo()'
        tgop = tgo;
        // canceling initialization flag
        ipas_flag = 0;
    }
    //-------------------------------------------------------------------------
}
///////////////////////////////////////////////////////////////////////////////
// LTG thrust integrals
//
// Parameter output:
//    s_igrl = thrust integral, postion - m
//    j_igrl = thrust integral, velocit*time - m
//    q_igrl = thrust integral, position*time - m*s
//    h_igrl = thrust integral, velocity*time^2 - m*s
//    p_igrl = thrust integral, position*time^2 - m*s^2
//    j_over_l= time remaining - s
//    tlam = time of thrust integration - s
//    qprime = conditioned q-integral - m*s
// Parameter input:
//    nst = n-th stage number - ND
//    num_stages = number of stages (input) - ND
//    BURNTN(3) = burn time duration left in n-th stage - s
//    L_IGRLN(3) = velocity to-be-gained by n-th stage at 'time_ltg' - m/s
//    TGON(3) = burn durations remaining, summed over n-th and previous
//              stages, towards reaching end-state - s
//    TAUN(3) = characteristic time of n-th stage - s
//    VEXN(3) = exhaust velocity of n-th stage - m/s
//    l_igrl = L-integral (velocity to be gained - m/s)
//    time_ltg = time clock started when LTG is engaged -s
//
// 040325 Converted from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Guidance::guidance_ltg_igrl(double &s_igrl,
                              double &j_igrl,
                              double &q_igrl,
                              double &h_igrl,
                              double &p_igrl,
                              double &j_over_l,
                              double &tlam,
                              double &qprime,
                              int nst,
                              int nstmax,
                              Matrix BURNTN,
                              Matrix L_IGRLN,
                              Matrix TGON,
                              Matrix TAUN,
                              Matrix VEXN,
                              double l_igrl,
                              double time_ltg)
{
    // local variables
    double ls_igrl(0);

    // calculating integrals for remaining boost stages
    for (int i = nst - 1; i < nstmax; i++) {
        double tb = BURNTN[i];
        double tga = TGON[i];
        double dummy = TAUN[i];
        double x = tb / dummy;

        double a1(0);
        if (x == 2)
            // factor 1.001 used to prevent division by zero
            a1 = 1 / (1 - 0.5 * x * (1.001));
        else
            a1 = 1 / (1 - 0.5 * x);

        double a2(0);
        if (x == 1) {
            // x=1 occurs when in '_tgo()':
            // BURNTN[i-1]=TAUN[i-1]*(1-exp(-almx/dum3)); the exponent is very
            // large
            //  and therefore BURNTN[i-1]=TAUN[i-1] -> caused by non-convergence
            //  of solution
            std::cerr
                << " *** LTG Terminator: end-state cannot be reached *** \n";
            //exit(1); //XXX: random exitpoint
        } else {
            a2 = 1 / (1 - x);
        }

        double dummo = TAUN[i];
        double aa =
            VEXN[i] / dummo;  // longitudinal acceleration of booster - m's^2

        double ll_igrl = L_IGRLN[i];
        double a1x = 4 * a1 - a2 - 3;
        double a2xsq = 2 * a2 - 4 * a1 + 2;
        double sa = (aa * tb * tb / 2) * (1 + a1x / 3 + a2xsq / 6);
        double ha = (aa * tb * tb * tb / 3) * (1. + a1x * 0.75 + a2xsq * 0.6);
        double ja = (aa * tb * tb / 2) * (1 + a1x * (2 / 3) + a2xsq / 2);
        double qa = (aa * tb * tb * tb / 6) * (1 + a1x / 2 + a2xsq * 0.3);
        double pa =
            (aa * tb * tb * tb * tb / 12) * (1 + a1x * 0.6 + a2xsq * 0.4);

        // if not the current stage
        if (i != nst - 1) {
            double t1 = TGON[i - 1];
            ha = ha + 2 * t1 * ja + t1 * t1 * ll_igrl;
            ja = ja + t1 * ll_igrl;
            pa = pa + 2 * t1 * qa + t1 * t1 * sa;
            qa = qa + t1 * sa;
        }
        ha = ja * tga - qa;
        sa = sa + ls_igrl * tb;
        qa = qa + j_igrl * tb;
        pa = pa + h_igrl * tb;
        s_igrl = s_igrl + sa;
        q_igrl = q_igrl + qa;
        p_igrl = p_igrl + pa;
        h_igrl = h_igrl + ha;
        ls_igrl = ls_igrl + ll_igrl;
        j_igrl = j_igrl + ja;
    }
    j_over_l = j_igrl / l_igrl;
    tlam = time_ltg + j_over_l;
    qprime = q_igrl - s_igrl * j_over_l;
}

///////////////////////////////////////////////////////////////////////////////
// Turning rate and thrust vector calculations
//
// Parameter output:
//    ULAM(3) = unit thrust vector in direction of VGO - ND
//    LAMD(3) = derivative of unit thrust vector - 1/s
//    RGO(3) = range-to-go - m
// Parameter input/output:
//    ipas2_flag = initialization flag - ND
// Parameter input:
//    VGO(3) = velocity still to be gained - m/s
//    ipas2_flag = initialization flag - ND
//    s_igrl = thrust integral, postion - m
//    q_igrl = thrust integral, position*time - m*s
//    j_over_l = time remaining - s
//    lamd_limit = LAMD components cannot exceed this limit (input) - 1/s
//    vgom = velocity to be gained - m/s
//    time_ltg = time clock started when LTG is engaged -s
// Parameter throughput to 'guidance_ltg_trate_rtgo(()':
//    tgo = time-to-go - s
//    tgop = previous time-to-go calculation - s
//    SDII(3) = desired range, defined in '_crct()' - m
//    SBIIC(3) = INS vehicle inertial position - m
//    VBIIC(3) = INS vehicle inertial velosicy - m/s
//    RBIAS(3) = position bias vector, calculated in '_pdct()' - m
//    UD(3) = unit vector of SPII and SDII - ND
//    UY(3) = unit vector normal to trajectory plane - ND
//    UZ(3) = unit vector in traj plane,
//            normal to desired inertial pos - ND
// Parameter modified in 'guidance_ltg_trate_rtgo(()':
//    RGRAV(3) = Postion loss due to gravity (modified in '_rtgo()') - m
//
// 040325 Converted from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Guidance::guidance_ltg_trate(Matrix &ULAM,
                               Matrix &LAMD,
                               Matrix &RGO,
                               int &ipas2_flag,
                               Matrix VGO,
                               double s_igrl,
                               double q_igrl,
                               double j_over_l,
                               double lamd_limit,
                               double vgom,
                               double time_ltg,
                               double tgo,
                               double tgop,
                               Matrix SDII,
                               Matrix SBIIC,
                               Matrix VBIIC,
                               Matrix RBIAS,
                               Matrix UD,
                               Matrix UY,
                               Matrix UZ,
                               Matrix &RGRAV)
{
    //-------------------------------------------------------------------------
    // return if velocity-to-be-gained is zero
    if (vgom == 0)
        return;

    // unit thrust vector in VGO direction
    ULAM = VGO.univec3();
    LAMD.zero();

    // initializing RGO at first pass
    if (ipas2_flag) {
        ipas2_flag = 0;
        RGO = ULAM * s_igrl;
    }
    // calling function to get range-to-go (SBII_GO)
    guidance_ltg_trate_rtgo(RGO,    // output
                            RGRAV,  // input-output
                            tgo, tgop, SDII, SBIIC, VBIIC, RBIAS, ULAM, UD, UY,
                            UZ, s_igrl);  // input
    double denom = (q_igrl - s_igrl * j_over_l);
    if (denom != 0)
        LAMD = (RGO - ULAM * s_igrl) * (1 / denom);
    else
        LAMD.zero();

    // setting limits on LAMD
    lamd = LAMD.absolute();
    if (lamd >= lamd_limit) {
        Matrix ULMD = LAMD.univec3();
        LAMD = ULMD * lamd_limit;
    }
    // diagnostics:
    lamd = LAMD.absolute();
    //-------------------------------------------------------------------------
}
///////////////////////////////////////////////////////////////////////////////
// Range-to-go calculation, called from 'guidance_ltg_trate()'
// * downrange is left free
// * corrects for gravity effects and adds bias corrections
//
// Parameter output:
//    RGO(3) = range-to-go vector - m
// Parameter input/output:
//    RGRAV(3) = Postion loss due to gravity - m
// Parameter input:
//    tgo = time-to-go - s
//    tgop = previous time-to-go calculation - s
//    VGO(3) = velocity to be achieved - m/s
//    SDII(3) = desired range, defined in '_crct()' - m
//    SBIIC(3) = INS vehicle inertial position - m
//    VBIIC(3) = INS vehicle inertial velosicy - m/s
//    RBIAS(3) = position bias vector, calculated in '_pdct()' - m
//    ULAM(3) = unit thrust vector - ND
//    UD(3) = unit vector of SPII and SDII - ND
//    UY(3) = unit vector normal to trajectory plane - ND
//    UZ(3) = unit vector in traj plane, normal to desired inertial pos - ND
//    s_igrl = thrust integral, postion - m
//
// 040326 Converted from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Guidance::guidance_ltg_trate_rtgo(Matrix &RGO,
                                    Matrix &RGRAV,
                                    double tgo,
                                    double tgop,
                                    Matrix SDII,
                                    Matrix SBIIC,
                                    Matrix VBIIC,
                                    Matrix RBIAS,
                                    Matrix ULAM,
                                    Matrix UD,
                                    Matrix UY,
                                    Matrix UZ,
                                    double s_igrl)
{
    // correcting range-due-to-gravity
    RGRAV = RGRAV * (tgo / tgop) * (tgo / tgop);
    Matrix RGO_LOCAL = SDII - (SBIIC + VBIIC * tgo + RGRAV) - RBIAS;

    // calculating range-to-go vector component in (UD,UY)-plane
    double rgox = RGO_LOCAL ^ UD;
    double rgoy = RGO_LOCAL ^ UY;
    Matrix RGOXY = UD * rgox + UY * rgoy;

    // replacing down-range z-component
    double num = RGOXY ^ ULAM;
    double denom = ULAM ^ UZ;
    if (denom == 0) {
        std::cerr
            << " *** Warning: divide by zero in 'guidance_ltg_trate_rtgo'; "
               "previous 'RGO' used *** \n";
        return;
    }
    double rgoz = (s_igrl - num) / denom;

    //(RGO is in the direction of ULAM with magnitude of s_igrl)
    RGO = RGOXY + UZ * rgoz;
}
///////////////////////////////////////////////////////////////////////////////
// End-state predictor calculations
// * corrects for gravity effects and adds bias corrections
//
// Parameter output:
//    SPII(3) = predicted inertial position vector - m
//    VPII(3) = predicted inertial velocity vector - m/s
//    RGRAV(3) = position loss due to gravity - m
//    RBIAS(3) = position bias -> used in '_rtgo()' - m
// Parameter input:
//    LAMD(3) = derivative of unit thrust vector - 1/s
//    ULAM(3) = unit thrust vector in direction of VGO - ND
//    l_igrl = L-integral (velocity to be gained - m/s)
//    s_igrl = thrust integral, postion - m
//    j_igrl = thrust integral, velocit*time - m
//    q_igrl = thrust integral, position*time - m*s
//    h_igrl = thrust integral, velocity*time^2 - m*s
//    p_igrl = thrust integral, position*time^2 - m*s^2
//    j_over_l= time remaining - s
//    qprime = conditioned q-integral - m*s
//    SBIIC(3) = INS vehicle inertial position - m
//    VBIIC(3) = INS vehicle inertial velosicy - m/s
//    RGO(3) = range-to-go - m
//    tgo = time-to-go - s
//
// 040325 Converted from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Guidance::guidance_ltg_pdct(Matrix &SPII,
                              Matrix &VPII,
                              Matrix &RGRAV,
                              Matrix &RBIAS,
                              Matrix LAMD,
                              Matrix ULAM,
                              double l_igrl,
                              double s_igrl,
                              double j_igrl,
                              double q_igrl,
                              double h_igrl,
                              double p_igrl,
                              double j_over_l,
                              double qprime,
                              Matrix SBIIC,
                              Matrix VBIIC,
                              Matrix RGO,
                              double tgo)
{
    // local variables
    Matrix SBIIC2(3, 1);
    Matrix VBIIC2(3, 1);

    // velocity gained due to thrust
    double lmdsq = LAMD ^ LAMD;
    Matrix VTHRUST =
        ULAM *
        (l_igrl - 0.5 * lmdsq * (h_igrl - j_igrl * j_over_l));  // Jackson, p.8

    // displacement gained due to thrust
    Matrix RTHRUST =
        ULAM *
            (s_igrl - 0.5 * lmdsq * (p_igrl - j_over_l * (q_igrl + qprime))) +
        LAMD * qprime;  // Jackson, p.8

    // bias used in '-rtgo()'
    RBIAS = RGO - RTHRUST;

    // offsetting SBIIC, VBIIC to SBIIC1, VBIIC1 for gravity calculations
    Matrix SBIIC1 =
        SBIIC - RTHRUST * 0.1 - VTHRUST * (tgo / 30);  // Jackson, p.23
    Matrix VBIIC1 =
        VBIIC + RTHRUST * (1.2 / tgo) - VTHRUST * 0.1;  // Jackson, p.23

    // calling Kepler utility to project to end state (two options available)
    int flag = cad_kepler(SBIIC2, VBIIC2, SBIIC1, VBIIC1, tgo);
    //    int flag=cad_kepler1(SBIIC2,VBIIC2,SBIIC1,VBIIC1,tgo);
    if (flag) {
        std::cerr
            << " *** Warning: bad Kepler projection in 'guidance_ltg_pdct()' "
               "*** \n";
    }
    // gravity corrections
    Matrix VGRAV = VBIIC2 - VBIIC1;
    RGRAV = SBIIC2 - SBIIC1 - VBIIC1 * tgo;

    // predicted state with gravity and thrust corrections
    SPII = SBIIC + VBIIC * tgo + RGRAV + RTHRUST;
    VPII = VBIIC + VGRAV + VTHRUST;
}
///////////////////////////////////////////////////////////////////////////////
// End-state corrector calculations
// * first called for intialization in 'guidance_ltg()'
//
// Parameter output:
//    SDII(3) = desired inertial position, defined here - m
//    UD(3) = unit vector of SPII and SDII - ND
//    UY(3) = unit vector normal to trajectory plane - ND
//    UZ(3) = unit vector in traj plane, normal to desired inertial pos - ND
//    VMISS(3) = velocity miss - m/s
//
// Parameter input/output:
//    VGO(3) = velocity to be achieved - m/s
//
// Parameter input:
//    dbi_desired = desired orbital end position (input) - m
//    dvbi_desired = desired end velocity (input) - m/s
//    SPII(3) = predicted inertial position vector - m
//    VPII(3) = predicted inertial velocity vector - m/s
//    thtvdx_desired = desired flight path angle - deg
//    SBIIC(3) = INS vehicle inertial position - m
//    VBIIC(3) = INS vehicle inertial velosicy - m/s
//
// 040325 Converted from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Guidance::guidance_ltg_crct(Matrix &SDII,
                              Matrix &UD,
                              Matrix &UY,
                              Matrix &UZ,
                              Matrix &VMISS,
                              Matrix &VGO,
                              double dbi_desired,
                              double dvbi_desired,
                              double thtvdx_desired,
                              Matrix SPII,
                              Matrix VPII,
                              Matrix SBIIC,
                              Matrix VBIIC)
{
    // local variables
    Matrix VDII(3, 1);

    // desired position
    UD = SPII.univec3();
    SDII = UD * dbi_desired;

    // generating unit base vectors ('%' overloaded Matrix operator)
    UY = VBIIC % SBIIC;
    UZ = UD % UY;

    // velocity-to-be-gained
    VDII = (UD * sin(thtvdx_desired * RAD) + UZ * cos(thtvdx_desired * RAD)) *
           dvbi_desired;
    VMISS = VPII - VDII;
    VGO = VGO - VMISS;

    // diagnostics:
    // displacement of P wrt D (both lie always on the UD vector)
    double dpi = SPII.absolute();
    double ddi = SDII.absolute();
    dpd = dpi - ddi;
    // displacement of vehicle B wrt the desired end-point D
    double dbi = SBIIC.absolute();
    dbd = dbi - ddi;

    //-------------------------------------------------------------------------
}

Matrix  Guidance::get_UTBC() {
    Matrix UTBC(3, 1);
    UTBC.build_vec3(utbc);
    return UTBC;
}
double  Guidance::get_alphacomx() { return alphacomx; }
double  Guidance::get_betacomx() { return betacomx; }

void Guidance::set_degree(double a, double b) {
    alphacomx = a;
    betacomx = b;
}
