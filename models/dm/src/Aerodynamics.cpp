#include "Aerodynamics.hh"

AeroDynamics::AeroDynamics(Kinematics &kine,  Environment &env,
                           Propulsion &prop,  _Euler_     &eul,
                           Newton     &newt,  TVC         &t)
    :   kinematics(&kine), environment(&env), propulsion(&prop),
        euler(&eul), newton(&newt), tvc(&t)
{

}

AeroDynamics::AeroDynamics(const AeroDynamics& other)
    :   kinematics(other.kinematics), environment(other.environment),
        propulsion(other.propulsion), euler(other.euler),
        newton(other.newton), tvc(other.tvc)
{
    this->aerotable = other.aerotable;

    this->xcg_ref = other.xcg_ref;
    this->alplimx = other.alplimx;
    this->alimitx = other.alimitx;
    this->refa = other.refa;
    this->refd = other.refd;
    this->dyb = other.dyb;
    this->dma = other.dma;
    this->dnb = other.dnb;
    this->dnd = other.dnd;
    this->dmq = other.dmq;
    this->dnr = other.dnr;
    this->dmde = other.dmde;
    this->dndr = other.dndr;
    this->gymax = other.gymax;
    this->dla = other.dla;
    this->cy = other.cy;
    this->cll = other.cll;
    this->clm = other.clm;
    this->cln = other.cln;
    this->cx = other.cx;
    this->cz = other.cz;

    this->ca0 = other.ca0;
    this->caa = other.caa;
    this->cn0 = other.cn0;
    this->clm0 = other.clm0;
    this->clmq = other.clmq;
    this->cla = other.cla;
    this->clde = other.clde;
    this->cyb = other.cyb;
    this->cydr = other.cydr;
    this->cllda = other.cllda;
    this->cllp = other.cllp;
    this->cma = other.cma;
    this->cmde = other.cmde;
    this->cmq = other.cmq;
    this->cnb = other.cnb;
    this->cndr = other.cndr;
    this->cnr = other.cnr;
    this->stmarg_yaw = other.stmarg_yaw;
    this->stmarg_pitch = other.stmarg_pitch;
    this->dlde = other.dlde;
    this->dydr = other.dydr;
    this->dllp = other.dllp;
    this->dllda = other.dllda;
    this->realp1 = other.realp1;
    this->realp2 = other.realp2;
    this->wnp = other.wnp;
    this->zetp = other.zetp;
    this->rpreal = other.rpreal;
    this->realy1 = other.realy1;
    this->realy2 = other.realy2;
    this->wny = other.wny;
    this->zety = other.zety;
    this->ryreal = other.ryreal;
    this->gnavail = other.gnavail;
    this->gyavail = other.gyavail;
    this->gnmax = other.gnmax;
}

AeroDynamics& AeroDynamics::operator=(const AeroDynamics& other){
    if(&other == this)
        return *this;

    this->kinematics = other.kinematics;
    this->environment = other.environment;
    this->propulsion = other.propulsion;
    this->euler = other.euler;
    this->newton = other.newton;
    this->tvc = other.tvc;

    this->aerotable = other.aerotable;

    this->xcg_ref = other.xcg_ref;
    this->alplimx = other.alplimx;
    this->alimitx = other.alimitx;
    this->refa = other.refa;
    this->refd = other.refd;
    this->dyb = other.dyb;
    this->dma = other.dma;
    this->dnb = other.dnb;
    this->dnd = other.dnd;
    this->dmq = other.dmq;
    this->dnr = other.dnr;
    this->dmde = other.dmde;
    this->dndr = other.dndr;
    this->gymax = other.gymax;
    this->dla = other.dla;
    this->cy = other.cy;
    this->cll = other.cll;
    this->clm = other.clm;
    this->cln = other.cln;
    this->cx = other.cx;
    this->cz = other.cz;

    this->ca0 = other.ca0;
    this->caa = other.caa;
    this->cn0 = other.cn0;
    this->clm0 = other.clm0;
    this->clmq = other.clmq;
    this->cla = other.cla;
    this->clde = other.clde;
    this->cyb = other.cyb;
    this->cydr = other.cydr;
    this->cllda = other.cllda;
    this->cllp = other.cllp;
    this->cma = other.cma;
    this->cmde = other.cmde;
    this->cmq = other.cmq;
    this->cnb = other.cnb;
    this->cndr = other.cndr;
    this->cnr = other.cnr;
    this->stmarg_yaw = other.stmarg_yaw;
    this->stmarg_pitch = other.stmarg_pitch;
    this->dlde = other.dlde;
    this->dydr = other.dydr;
    this->dllp = other.dllp;
    this->dllda = other.dllda;
    this->realp1 = other.realp1;
    this->realp2 = other.realp2;
    this->wnp = other.wnp;
    this->zetp = other.zetp;
    this->rpreal = other.rpreal;
    this->realy1 = other.realy1;
    this->realy2 = other.realy2;
    this->wny = other.wny;
    this->zety = other.zety;
    this->ryreal = other.ryreal;
    this->gnavail = other.gnavail;
    this->gyavail = other.gyavail;
    this->gnmax = other.gnmax;

    return *this;
}


void AeroDynamics::initialize(){
}

void AeroDynamics::load_aerotable(char* filename){
    aerotable = Datadeck(filename);
}

void AeroDynamics::set_xcg_ref(double in) { xcg_ref = in; }
void AeroDynamics::set_alplimx(double in) { alplimx = in; }
void AeroDynamics::set_alimitx(double in) { alimitx = in; }
void AeroDynamics::set_refa(double in) { refa = in; }
void AeroDynamics::set_refd(double in) { refd = in; }

void AeroDynamics::calculate_aero(double int_step){
    /* only calculate when rocket liftoff */
    if(newton->get_liftoff() == 1){
        //Input data from other module//////////////////
        double alppx  = kinematics->get_alppx();
        double phipx  = kinematics->get_phipx();
        double alphax = kinematics->get_alphax();
        double betax  = kinematics->get_betax();

        double rho    = environment->get_rho();
        double vmach  = environment->get_vmach();
        double pdynmc = environment->get_pdynmc();
        double tempk  = environment->get_tempk();
        double dvba   = environment->get_dvba();

        double ppx    = euler->get_ppx();
        double qqx    = euler->get_qqx();
        double rrx    = euler->get_rrx();

        double alt    = newton->get_alt();

        enum Propulsion::THRUST_TYPE thrust_state
                    = propulsion->get_thrust_state();
        double vmass  = propulsion->get_vmass();
        double xcg    = propulsion->get_xcg();
        ////////////////////////////////////////////////////
        double cn = 0;

        //transforming body rates from body -> aeroballistic coord.
        double phip  = phipx * RAD;
        double cphip = cos(phip);
        double sphip = sin(phip);
        double qqax  = qqx * cphip - rrx * sphip;
        double rrax  = qqx * sphip + rrx * cphip;

        //looking up axial force coefficients
        ca0  = aerotable.look_up("ca0_vs_mach",  vmach);
        caa  = aerotable.look_up("caa_vs_mach",  vmach);
        double ca0b = aerotable.look_up("ca0b_vs_mach", vmach);
        //axial force coefficient
        double ca = ca0 + caa * alppx + (thrust_state != Propulsion::NO_THRUST) * ca0b;

        //looking up normal force coefficients in aeroballistic coord
        //normal force coefficient
        double cna = cn0 = aerotable.look_up("cn0_vs_mach_alpha", vmach, alppx);

        //looking up pitching moment coefficients in aeroballistic coord
        clm0 = aerotable.look_up("clm0_vs_mach_alpha", vmach, alppx);
        clmq = aerotable.look_up("clmq_vs_mach", vmach);

        //pitching moment coefficient
        double clmaref = clm0 + clmq * qqax * refd / (2. * dvba);
        double clma = clmaref - cna * (xcg_ref - xcg) / refd;

        //Non-dimensional derivatives
        //look up coeff at +- 3 deg, but not beyond tables
        double alplx = 3.0;
        double alpmx = -3.0;
        if(alpmx < 0.) alpmx = 0.0;

        //calculating normal force dim derivative wrt alpha 'cla'
        double cn0p = aerotable.look_up("cn0_vs_mach_alpha", vmach, alplx);
        double cn0m = aerotable.look_up("cn0_vs_mach_alpha", vmach, alpmx);
        //cout<<cn0p<<'\t'<<vmach<<'\t'<<alplx<<endl;
        //
        //replacing value from previous cycle, only if within max alpha limit
        if(alplx < alplimx)
            cla = (cn0p - cn0m) / (alplx - alpmx);
        //calculating pitch moment dim derivative wrt alpha 'cma'
        double clm0p = aerotable.look_up("clm0_vs_mach_alpha", vmach, alplx);
        double clm0m = aerotable.look_up("clm0_vs_mach_alpha", vmach, alpmx);
        //replacing value from previous cycle, only if within max alpha limit
        if(alppx < alplimx)
            cma = (clm0p - clm0m) / (alplx - alpmx) - cla * (xcg_ref - xcg) / refd;

        //converting force and moment coeff to body axes
        //force coefficients in body axes
        cx = -ca;
        cy = -cna * sphip;
        cz = -cna * cphip;
        //moment coefficient in body axes
        cll = 0;
        clm = clma * cphip;
        cln = -clma * sphip;

        //calculate load factor available for max alpha
        //looking up normal force coefficients in aeroballistic coord
        double cn0mx = aerotable.look_up("cn0_vs_mach_alpha", vmach, alplimx);

        double anlmx  = cn0mx * pdynmc * refa;
        double weight = vmass * AGRAV;
        gnmax = anlmx / weight;
        if(gnmax >= alimitx) gnmax = alimitx;

        double aloadn = cn * pdynmc * refa;
        double gng    = aloadn / weight;
        gnavail = gnmax-gng;
        //same load factor in yaw plane
        gymax   = gnmax;
        gyavail = gnavail;

        //converting output to be compatible with 'aerodynamics_der()'
        //force
        clde = 0.0;
        cyb  = -cla;
        cydr = 0.0;
        //roll
        cllda = 0.0;
        cllp  = 0.0;
        //pitch
        cmde = 0.0;
        cmq  = clmq;
        //yaw
        cnb  = -cma;
        cndr = 0.0;
        cnr  = clmq;

        aerodynamics_der();
    }
}



void AeroDynamics::aerodynamics_der(){
    double vmach  = environment->get_vmach();
    double pdynmc = environment->get_pdynmc();
    double dvba   = environment->get_dvba();
    double vmass  = propulsion->get_vmass();
    double xcg    = propulsion->get_xcg();
    double thrust = propulsion->get_thrust();
    int    mtvc   = tvc->get_mtvc();
    double gtvc   = tvc->get_gtvc();
    double parm   = tvc->get_parm();

    arma::mat33 IBBB = propulsion->get_IBBB();

    //MOI components
    double ibbb11 = IBBB(0,0);
    double ibbb22 = IBBB(1,1);
    double ibbb33 = IBBB(2,2);
    //Dimensional derivatives for pitch plane (converted to 1/rad where required)
    double duml = (pdynmc * refa / vmass) / RAD;
    dla = duml * cla;
    dlde = duml * clde;
    dnd = duml * cn0;
    double dumm = pdynmc * refa * refd / ibbb22;
    dma = dumm * cma / RAD;
    dmq = dumm * (refd / (2 * dvba)) * cmq;
    dmde = dumm * cmde / RAD;

    //Dimensional derivatives in plane (converted to 1/rad where required)
    double dumy = pdynmc*refa/vmass;
    dyb = dumy * cyb / RAD;
    dydr = dumy * cydr / RAD;
    double dumn = pdynmc * refa * refd / ibbb33;
    dnb = dumn * cnb / RAD;
    dnr = dumn * (refd / (2 * dvba)) * cnr;
    dndr = dumn * cndr / RAD;

    //Dimensional derivatives in roll (converted to 1/rad where required)
    double dumll = pdynmc*refa*refd/ibbb11;
    dllp = dumll * (refd / (2 * dvba)) * cllp;
    dllda = dumll * cllda / RAD;

    //TVC control derivatives
    if(mtvc == 1 || mtvc == 2 || mtvc == 3){
        //pitch plane
        dlde = gtvc * thrust / vmass;
        dmde = -(parm - xcg) * gtvc * thrust / IBBB(2,2);

        //yaw plane
        dydr = dlde;
        dndr = dmde;
    }
    //static margin in pitch (per chord length 'refd')
    if(cla) stmarg_pitch = -cma / cla;

    //static margin in yaw (per span length 'refd')
    if(cyb) stmarg_yaw = -cnb / cyb;

    //diagnostics: pitch plane roots
    double a11 = dmq;
    double a12(0);
    if(dla)
        a12 = dma / dla;
    double a21 = dla;
    double a22 = -dla / dvba;

    double arg = pow((a11 + a22), 2) - 4. * (a11 * a22 - a12 * a21);
    if(arg >= 0.)
    {
        wnp = 0.;
        zetp = 0.;
        double dum = a11+a22;
        realp1 = (dum + sqrt(arg)) / 2;
        realp2 = (dum - sqrt(arg)) / 2;
        rpreal = (realp1 + realp2) / 2;
    }
    else
    {
        realp1 = 0.;
        realp2 = 0.;
        wnp = sqrt(a11 * a22 - a12 * a21);
        zetp = -(a11 + a22) / (2 * wnp);
        rpreal = -zetp * wnp;
    }
    //diagnostics: yaw plane roots
    a11 = dnr;
    if(dyb)
        a12 = dnb / dyb;
    else
        a12 = 0;
    a21 = -dyb;
    a22 = dyb / dvba;
    arg = pow((a11 + a22), 2) - 4 * (a11 * a22 - a12 * a21);
    if(arg >= 0.)
    {
        wny = 0.;
        zety = 0.;
        double dum = a11 + a22;
        realy1 = (dum + sqrt(arg)) / 2;
        realy2 = (dum - sqrt(arg)) / 2;
        ryreal = (realy1 + realy2) / 2;
    }
    else
    {
        realy1 = 0.;
        realy2 = 0.;
        wny = sqrt(a11 * a22 - a12 * a21);
        zety = -(a11 + a22) / (2. * wny);
        ryreal = -zety * wny;
    }
}

double AeroDynamics::get_dyb() { return dyb; }
double AeroDynamics::get_dma() { return dma; }
double AeroDynamics::get_dnb() { return dnb; }
double AeroDynamics::get_dnd() { return dnd; }
double AeroDynamics::get_dmq() { return dmq; }
double AeroDynamics::get_dnr() { return dnr; }
double AeroDynamics::get_dmde() { return dmde; }
double AeroDynamics::get_dndr() { return dndr; }
double AeroDynamics::get_gymax() { return gymax; }
double AeroDynamics::get_dla() { return dla; }
double AeroDynamics::get_refa() { return refa; }
double AeroDynamics::get_refd() { return refd; }
double AeroDynamics::get_cy() { return cy; }
double AeroDynamics::get_cll() { return cll; }
double AeroDynamics::get_clm() { return clm; }
double AeroDynamics::get_cln() { return cln; }
double AeroDynamics::get_cx() { return cx; }
double AeroDynamics::get_cz() { return cz; }
double AeroDynamics::get_dlde() { return dlde; }
