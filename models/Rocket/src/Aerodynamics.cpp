#include "Aerodynamics.hh"

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

int AeroDynamics::set_maero(int in) { maero = in; }
double AeroDynamics::set_xcg_ref(double in) { xcg_ref = in; }
double AeroDynamics::set_alplimx(double in) { alplimx = in; }
double AeroDynamics::set_alimitx(double in) { alimitx = in; }
double AeroDynamics::set_refa(double in) { refa = in; }
double AeroDynamics::set_refd(double in) { refd = in; }

void AeroDynamics::initialize(Kinematics *kine, Environment *env, Propulsion *prop, _Euler_ *eul, Newton *newt, TVC *t)
{
    kinematics = kine;
    environment = env;
    propulsion = prop;
    euler = eul;
    newton = newt;
    tvc = t;

    read_tables("auxiliary/aero_table.txt",aerotable);
}

void AeroDynamics::calculate_aero(double int_step, Datadeck &aerotable)
{
    //Input data from other module//////////////////
    double alppx=kinematics->alppx;
    double phipx=kinematics->phipx;
    double alphax=kinematics->alphax;
    double betax=kinematics->betax;
    double rho=environment->get_rho();
    double vmach=environment->get_vmach();
    double pdynmc=environment->get_pdynmc();
    double tempk=environment->get_tempk();
    double dvba=environment->get_dvba();
    double ppx=euler->get_ppx();
    double qqx=euler->get_qqx();
    double rrx=euler->get_rrx();
    double alt=newton->get_alt();
    double mprop=propulsion->mprop;
    double vmass=propulsion->vmass;
    double xcg=propulsion->xcg;
    ////////////////////////////////////////////////////
    int thrust_on=false;
    double ca0b(0);
    double cna(0);
    double clma(0);
    double ca(0);
    double cn0mx(0);
    double cn(0);

    //transforming body rates from body -> aeroballistic coord.
    double phip=phipx*RAD;
    double cphip=cos(phip);
    double sphip=sin(phip);
    double qqax=qqx*cphip-rrx*sphip;
    double rrax=qqx*sphip+rrx*cphip;


    //looking up axial force coefficients
    if(maero==13){

        ca0=aerotable.look_up("ca0slv3_vs_mach",vmach);
        caa=aerotable.look_up("caaslv3_vs_mach",vmach);
        ca0b=aerotable.look_up("ca0bslv3_vs_mach",vmach);
    }else if(maero== 12){
        ca0=aerotable.look_up("ca0slv2_vs_mach",vmach);
        caa=aerotable.look_up("caaslv2_vs_mach",vmach);
        ca0b=aerotable.look_up("ca0bslv2_vs_mach",vmach);
    }else if(maero==11){
       ca0=aerotable.look_up("ca0slv2_vs_mach",vmach);
       caa=aerotable.look_up("caaslv2_vs_mach",vmach);
       ca0b=aerotable.look_up("ca0bslv2_vs_mach",vmach);
    }
    //axial force coefficient
    if(mprop) thrust_on=true;
    ca=ca0+caa*alppx+thrust_on*ca0b;

    //looking up normal force coefficients in aeroballistic coord
    if(maero==13){
       cn0=aerotable.look_up("cn0slv3_vs_mach_alpha",vmach,alppx);
    }else if (maero==12){
       cn0=aerotable.look_up("cn0slv2_vs_mach_alpha",vmach,alppx);
    }else if (maero==11){
       cn0=aerotable.look_up("cn0slv1_vs_mach_alpha",vmach,alppx);
    }
    //normal force coefficient
    cna=cn0;

    //looking up pitching moment coefficients in aeroballistic coord
    if (maero==13){
       clm0=aerotable.look_up("clm0slv3_vs_mach_alpha",vmach,alppx);
       clmq=aerotable.look_up("clmqslv3_vs_mach",vmach);
    }else if(maero==12){
       clm0=aerotable.look_up("clm0slv2_vs_mach_alpha",vmach,alppx);
       clmq=aerotable.look_up("clmqslv2_vs_mach",vmach);
    }else if(maero==11){
       clm0=aerotable.look_up("clm0slv1_vs_mach_alpha",vmach,alppx);
       clmq=aerotable.look_up("clmqslv1_vs_mach",vmach);
    }
    //pitching moment coefficient
    double clmaref=clm0+clmq*qqax*refd/(2.*dvba);
    clma=clmaref-cna*(xcg_ref-xcg)/refd;

    double alplx(0),alpmx(0);
    double cn0p(0),cn0m(0);
    double clm0p(0),clm0m(0);

    //Non-dimensional derivatives
    //look up coeff at +- 3 deg, but not beyond tables
    alplx=alppx+3.0;
    alpmx=alppx-3.0;
    if(alpmx<0.)alpmx=0.0;

    //calculating normal force dim derivative wrt alpha 'cla'
    if(maero==13){
       cn0p=aerotable.look_up("cn0slv3_vs_mach_alpha",vmach,alplx);
       cn0m=aerotable.look_up("cn0slv3_vs_mach_alpha",vmach,alpmx);
    }else if(maero==12){
       cn0p=aerotable.look_up("cn0slv2_vs_mach_alpha",vmach,alplx);
       cn0m=aerotable.look_up("cn0slv2_vs_mach_alpha",vmach,alpmx);
    }else if(maero==11){
       cn0p=aerotable.look_up("cn0slv1_vs_mach_alpha",vmach,alplx);
       cn0m=aerotable.look_up("cn0slv1_vs_mach_alpha",vmach,alpmx);
    }
    //cout<<cn0p<<'\t'<<vmach<<'\t'<<alplx<<endl;
    //replacing value from previous cycle, only if within max alpha limit
    if(alplx<alplimx)
        cla=(cn0p-cn0m)/(alplx-alpmx);
    //calculating pitch moment dim derivative wrt alpha 'cma'
    if(maero== 13){
       clm0p=aerotable.look_up("clm0slv3_vs_mach_alpha",vmach,alplx);
       clm0m=aerotable.look_up("clm0slv3_vs_mach_alpha",vmach,alpmx);
    }else if(maero==12){
       clm0p=aerotable.look_up("clm0slv2_vs_mach_alpha",vmach,alplx);
       clm0m=aerotable.look_up("clm0slv2_vs_mach_alpha",vmach,alpmx);
    }else if(maero==11){
       clm0p=aerotable.look_up("clm0slv1_vs_mach_alpha",vmach,alplx);
       clm0m=aerotable.look_up("clm0slv1_vs_mach_alpha",vmach,alpmx);
    }
    //replacing value from previous cycle, only if within max alpha limit
    if(alppx<alplimx)
        cma=(clm0p-clm0m)/(alplx-alpmx)-cla*(xcg_ref-xcg)/refd;

    //converting force and moment coeff to body axes
    //force coefficients in body axes
    cx=-ca;
    cy=-cna*sphip;
    cz=-cna*cphip;
    //moment coefficient in body axes
    cll=0;
    clm=clma*cphip;
    cln=-clma*sphip;

    //calculate load factor available for max alpha
    //looking up normal force coefficients in aeroballistic coord
    if(maero==13){

        cn0mx=aerotable.look_up("cn0slv3_vs_mach_alpha",vmach,alplimx);
    }else if(maero==12){
        cn0mx=aerotable.look_up("cn0slv2_vs_mach_alpha",vmach,alplimx);
    }else if(maero==11){
        cn0mx=aerotable.look_up("cn0slv1_vs_mach_alpha",vmach,alplimx);
    }
    double anlmx=cn0mx*pdynmc*refa;
    double weight=vmass*AGRAV;
    gnmax=anlmx/weight;
    if(gnmax>=alimitx)gnmax=alimitx;
    double aloadn=cn*pdynmc*refa;
    double gng=aloadn/weight;
    gnavail=gnmax-gng;
    //same load factor in yaw plane
    gymax=gnmax;
    gyavail=gnavail;

    //converting output to be compatible with 'aerodynamics_der()'
    //force
    clde=0.0;
    cyb=-cla;
    cydr=0.0;
    //roll
    cllda=0.0;
    cllp=0.0;
    //pitch
    cmde=0.0;
    cmq=clmq;
    //yaw
    cnb=-cma;
    cndr=0.0;
    cnr=clmq;


    aerodynamics_der();

}



void AeroDynamics::aerodynamics_der()
{
    double vmach=environment->get_vmach();
    double pdynmc=environment->get_pdynmc();
    double dvba=environment->get_dvba();
    double vmass=propulsion->vmass;
    double xcg=propulsion->xcg;
    double thrust=propulsion->thrust;
    int mtvc=tvc->mtvc;
    double gtvc=tvc->gtvc;
    double parm=tvc->parm;

    Matrix IBBB(3,3);
    IBBB.build_mat33(propulsion->ibbb);

    //MOI components
    double ibbb11=IBBB.get_loc(0,0);
    double ibbb22=IBBB.get_loc(1,1);
    double ibbb33=IBBB.get_loc(2,2);
    //Dimensional derivatives for pitch plane (converted to 1/rad where required)
    double duml=(pdynmc*refa/vmass)/RAD;
    dla=duml*cla;
    dlde=duml*clde;
    dnd=duml*cn0;
    double dumm=pdynmc*refa*refd/ibbb22;
    dma=dumm*cma/RAD;
    dmq=dumm*(refd/(2*dvba))*cmq;
    dmde=dumm*cmde/RAD;

    //Dimensional derivatives in plane (converted to 1/rad where required)
    double dumy=pdynmc*refa/vmass;
    dyb=dumy*cyb/RAD;
    dydr=dumy*cydr/RAD;
    double dumn=pdynmc*refa*refd/ibbb33;
    dnb=dumn*cnb/RAD;
    dnr=dumn*(refd/(2*dvba))*cnr;
    dndr=dumn*cndr/RAD;

    //Dimensional derivatives in roll (converted to 1/rad where required)
    double dumll=pdynmc*refa*refd/ibbb11;
    dllp=dumll*(refd/(2*dvba))*cllp;
    dllda=dumll*cllda/RAD;

    //TVC control derivatives
    if(mtvc==1||mtvc==2||mtvc==3){
        //pitch plane
        dlde=gtvc*thrust/vmass;
        dmde=-(parm-xcg)*gtvc*thrust/IBBB.get_loc(2,2);

        //yaw plane
        dydr=dlde;
        dndr=dmde;
    }
    //static margin in pitch (per chord length 'refd')
    if(cla) stmarg_pitch=-cma/cla;

    //static margin in yaw (per span length 'refd')
    if(cyb) stmarg_yaw=-cnb/cyb;

    //diagnostics: pitch plane roots
    double a11=dmq;
    double a12(0);
    if(dla)
        a12=dma/dla;
    double a21=dla;
    double a22=-dla/dvba;

    double arg=pow((a11+a22),2)-4.*(a11*a22-a12*a21);
    if(arg>=0.)
    {
        wnp=0.;
        zetp=0.;
        double dum=a11+a22;
        realp1=(dum+sqrt(arg))/2;
        realp2=(dum-sqrt(arg))/2;
        rpreal=(realp1+realp2)/2;
    }
    else
    {
        realp1=0.;
        realp2=0.;
        wnp=sqrt(a11*a22-a12*a21);
        zetp=-(a11+a22)/(2*wnp);
        rpreal=-zetp*wnp;
    }
    //diagnostics: yaw plane roots
    a11=dnr;
    if(dyb)
        a12=dnb/dyb;
    else
        a12=0;
    a21=-dyb;
    a22=dyb/dvba;
    arg=pow((a11+a22),2)-4*(a11*a22-a12*a21);
    if(arg>=0.)
    {
        wny=0.;
        zety=0.;
        double dum=a11+a22;
        realy1=(dum+sqrt(arg))/2;
        realy2=(dum-sqrt(arg))/2;
        ryreal=(realy1+realy2)/2;
    }
    else
    {
        realy1=0.;
        realy2=0.;
        wny=sqrt(a11*a22-a12*a21);
        zety=-(a11+a22)/(2.*wny);
        ryreal=-zety*wny;
    }
}
