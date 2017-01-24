#include "utility_header.hh"

#include "Environment.hh"
#include "sim_services/include/simtime.h"

double Environment::get_rho() { return rho; }
double Environment::get_vmach() { return vmach; }
double Environment::get_pdynmc() { return pdynmc; }
double Environment::get_tempk() { return tempk; }
double Environment::get_dvba() { return dvba; }
double Environment::get_grav() { return grav; }
double Environment::get_press() { return press; }

double* Environment::get_gravg_ptr() { return gravg; }
Matrix Environment::get_GRAVG() {
    Matrix GRAVG(3, 1);
    GRAVG.build_vec3(gravg);
    return GRAVG;
}
Matrix Environment::get_VAED() {
    Matrix VAED(3, 1);
    VAED.build_vec3(vaed);
    return VAED;
}

void Environment::initialize(Newton *newt, AeroDynamics *aero, Kinematics *kine)
{
    kinematics=kine;
    newton=newt;
    aerodynamics=aero;

    dvba=newton->get_dvbe();

    read_tables("auxiliary/weather_table.txt",weathertable);
}

void Environment::calculate_env(double int_step,Datadeck &weathertable)
{
    double dvw(0);
    Matrix GRAVG(3,1);
    Matrix VAED(vaed);
    Matrix VAEDS(vaeds);
    Matrix VAEDSD(vaedsd);
    Matrix VBED = newton->get_VBED();
    Matrix SBII(3,1);
    SBII.build_vec3(newton->get_IPos()[0],newton->get_IPos()[1],newton->get_IPos()[2]);
    double alt=newton->get_alt();
    //decoding the air switch
    int matmo=mair/100;
    int mturb=(mair-matmo*100)/10;
    int mwind=(mair-matmo*100)%10;

    GRAVG=cad_grav84(SBII,get_rettime());
    grav=GRAVG.absolute();

    //US 1976 Standard Atmosphere (public domain)
    if(matmo==0){
        atmosphere76(rho,press,tempk, alt);
        tempc=tempk-273.16;
        //speed of sound
        vsound=sqrt(1.4*RGAS*tempk);
    }
    //US 1976 Standard Atmosphere (NASA Marshall)
    if(matmo==1){
        double alt_km=alt/1000;
        int check=us76_nasa2002(alt_km,&rho,&press,&tempk,&vsound);
        tempc=tempk-273.16;
    }

    //tabular atmosphere from WEATHER_DECK
    if(matmo==2){

        rho=weathertable.look_up("density",alt);
        press=weathertable.look_up("pressure",alt);
        tempc=weathertable.look_up("temperature",alt);
        //speed of sound
        tempk=tempc+273.16;
        vsound=sqrt(1.4*RGAS*tempk);
    }
    //mach number
    vmach=fabs(dvba/vsound);

    //dynamic pressure
    pdynmc=0.5*rho*dvba*dvba;

    //wind options
    if(mwind>0){
        if(mwind==1)
            //constant wind
            dvw=dvae;

        if(mwind==2){
            //tabular wind from WEATHER_DECK
            dvw=weathertable.look_up("speed",alt);
            psiwdx=weathertable.look_up("direction",alt);
        }
        //wind components in geodetic coordinates
        Matrix VAED_RAW(3,1);
        VAED_RAW[0]=-dvw*cos(psiwdx*RAD);
        VAED_RAW[1]=-dvw*sin(psiwdx*RAD);
        VAED_RAW[2]=vaed3;

        //smoothing wind by filtering with time constant 'twind' sec
        Matrix VAEDSD_NEW=(VAED_RAW-VAEDS)*(1/twind);
        VAEDS=integrate(VAEDSD_NEW,VAEDSD,VAEDS,int_step);
        VAEDSD=VAEDSD_NEW;
        VAED=VAEDS;
    }
    //wind turbulence in normal-load plane
    if(mturb==1){
        Matrix VTAD=environment_dryden(dvba,int_step);
        VAED=VTAD+VAEDS;
    }
    //flight conditions
    Matrix VBAD=VBED-VAED;
    dvba=VBAD.absolute();

    //mach number
    vmach=fabs(dvba/vsound);
    //dynamic pressure
    pdynmc=0.5*rho*dvba*dvba;

    //update matrix elements
    VAED.fill(vaed);
    VAEDS.fill(vaeds);
    VAEDSD.fill(vaedsd);
    GRAVG.fill(gravg);


}



Matrix Environment::environment_dryden(double dvba,double int_step)
{
    Matrix VTAD(3,1);
    Matrix TBD(3,3);
    TBD.build_mat33(kinematics->tbd[0][0],kinematics->tbd[0][1],kinematics->tbd[0][2],
                    kinematics->tbd[1][0],kinematics->tbd[1][1],kinematics->tbd[1][2],
                    kinematics->tbd[2][0],kinematics->tbd[2][1],kinematics->tbd[2][2]);
    double alppx=kinematics->alppx;
    double phipx=kinematics->phipx;
    double value1;
    do
        value1=(double)rand()/RAND_MAX;
    while(value1==0);
    double value2=(double)rand()/RAND_MAX;
    gauss_value=(1/sqrt(int_step))*sqrt(2*log(1/value1))*cos(2*PI*value2);

    //filter, converting white gaussian noise into a time sequence of Dryden
    // turbulence velocity variable 'tau'  (One-dimensional cross-velocity Dryden spectrum)
    //integrating first state variable
    double taux1d_new=taux2;
    taux1=integrate(taux1d_new,taux1d,taux1,int_step);
    taux1d=taux1d_new;
    //integrating second state variable
    double vl=dvba/turb_length;
    double taux2d_new=-vl*vl*taux1-2*vl*taux2+vl*vl*gauss_value;
    taux2=integrate(taux2d_new,taux2d,taux2,int_step);
    taux2d=taux2d_new;
    //computing Dryden 'tau' from the two filter states ('2*PI' changed to 'PI' according to Pritchard)
    tau=turb_sigma*sqrt(1/(vl*PI))*(taux1+sqrt(3.)*taux2/vl);

    //inserting the turbulence into the load factor plane (aeroballistic 1A-3A plane)
    // and transforming into body coordinates VTAB=TBA*VTAA; VTAA=[0 0 tau]
    Matrix VTAB(3,1);
    VTAB[0]=-tau*sin(alppx*RAD);
    VTAB[1]=tau*sin(phipx*RAD)*cos(alppx*RAD);
    VTAB[2]=tau*cos(phipx*RAD)*cos(alppx*RAD);

    //turbulence in geodetic coordinates
    VTAD=~TBD*VTAB;

    return VTAD;
}
