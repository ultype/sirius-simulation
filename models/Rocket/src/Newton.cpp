#include "Newton.hh"
#include "trick_utils/math/include/quat_macros.h"
#include "trick_utils/math/include/matrix_macros.h"
#include "trick_utils/math/include/vector_macros.h"
#include "trick_utils/math/include/trick_math_proto.h"
#include "sim_services/include/simtime.h"

#include "utility_header.hh"

void Newton::default_data(){
}

void Newton::initialize(Kinematics *kine, _Euler_ *elr, Environment *env, Propulsion *prop, Forces *forc){
    kinematics = kine;
    euler = elr;
    environment = env;
    propulsion = prop;
    forces = forc;

    Matrix WBII(3, 3);
    double dbi(0);
    Matrix VBED(3, 1);
    Matrix SBII(3, 1);
    Matrix VBII(3, 1);
    double dvbi(0);
    double psivdx(0);
    double thtvdx(0);

    double psibdx=kinematics->psibdx;
    double thtbdx=kinematics->thtbdx;
    double phibdx=kinematics->phibdx;

    Matrix VBEB(3,1);
    Matrix POLAR(3,1);
    //Earth's angular velocity skew-symmetric matrix (3x3)
    Matrix WEII(3,3);
    WEII.assign_loc(0,1,-WEII3);
    WEII.assign_loc(1,0,WEII3);

    //converting geodetic lonx, latx, alt to SBII
    // XXX: time might skew
    SBII=cad_in_geo84(lonx*RAD,latx*RAD,alt,get_rettime());
    dbi=SBII.absolute();

    //building geodetic velocity VBED(3x1) from  alpha, beta, and dvbe
    double salp=sin(alpha0x*RAD);
    double calp=cos(alpha0x*RAD);
    double sbet=sin(beta0x*RAD);
    double cbet=cos(beta0x*RAD);
    double vbeb1=calp*cbet*dvbe;
    double vbeb2=sbet*dvbe;
    double vbeb3=salp*cbet*dvbe;
    VBEB.build_vec3(vbeb1,vbeb2,vbeb3);

    //building TBD
    Matrix TBD=mat3tr(psibdx*RAD,thtbdx*RAD,phibdx*RAD);
    //Geodetic velocity
    VBED=~TBD*VBEB;

    //building inertial velocity
    Matrix TDI=cad_tdi84(lonx*RAD,latx*RAD,alt,get_rettime());
    Matrix TGI=cad_tgi84(lonx*RAD,latx*RAD,alt,get_rettime());
    VBII=~TDI*VBED+WEII*SBII;
    dvbi=VBII.absolute();

    //calculating geodetic flight path angles (plotting initialization)
    POLAR=VBED.pol_from_cart();
    psivdx=DEG*POLAR.get_loc(1,0);
    thtvdx=DEG*POLAR.get_loc(2,0);

    //Update Matrix variables
    WEII.fill(weii);
    SBII.fill(IPos);
    VBII.fill(IVel);
    VBED.fill(vbed);
    TDI.fill(tdi);
    TGI.fill(tgi);
}

void Newton::calculate(double int_step){
    double lon(0);
    double lat(0);

    Matrix FSPB(3, 1);

    Matrix TDI(3, 3);
    Matrix TGI(3, 3);
    Matrix WEII(3, 3);
    TDI.build_mat33(tdi);
    TGI.build_mat33(tgi);
    WEII.build_mat33(weii);

    Matrix SBII(3, 1);
    Matrix VBII(3, 1);
    Matrix ABII(3, 1);
    SBII.build_vec3(IPos);
    VBII.build_vec3(IVel);
    ABII.build_vec3(IAccl);

    Matrix GRAVG(3,1);
    GRAVG.build_vec3(environment->gravg);

    Matrix TBI(3,3);
    Matrix FAPB(3,1);
    TBI.build_mat33(kinematics->tbi);
    FAPB.build_vec3(forces->fapb);

    Matrix FAP(3,1);
    FAP.build_vec3(forces->fap);

    double grav=environment->grav;
    double vmass=propulsion->vmass;

    FSPB=FAPB*(1./vmass);
    Matrix NEXT_ACC=~TBI*FSPB+~TGI*GRAVG;
    Matrix NEXT_VEL=integrate(NEXT_ACC,ABII,VBII,int_step);
    SBII=integrate(NEXT_VEL,VBII,SBII,int_step);
    ABII=NEXT_ACC;
    VBII=NEXT_VEL;
    dvbi=VBII.absolute();
    dbi=SBII.absolute();
    //calculate aero loss
    FAP=FAP*(1./vmass);
    aero_loss=aero_loss+FAP.absolute()*int_step;

    cad_geo84_in(lon,lat,alt,SBII,get_rettime());
    TDI=cad_tdi84(lon,lat,alt,get_rettime());
    TGI=cad_tgi84(lon,lat,alt,get_rettime());
    lonx=lon*DEG;
    latx=lat*DEG;
    //geographic velocity in geodetic axes VBED(3x1) and flight path angles
    Matrix VBED=TDI*(VBII-WEII*SBII);
    Matrix POLAR=VBED.pol_from_cart();
    dvbe=POLAR[0];
    psivdx=DEG*POLAR[1];
    thtvdx=DEG*POLAR[2];
    //calculate gravity loss
    gravity_loss2=gravity_loss2+grav*sin(thtvdx*RAD)*int_step;

    //T.M. of geographic velocity wrt geodetic coordinates
    Matrix TVD=mat2tr(psivdx*RAD,thtvdx*RAD);

    //diagnostics: acceleration achieved
    ayx=FSPB[1]/AGRAV;
    anx=-FSPB[2]/AGRAV;

    //ground track travelled (10% accuracy, usually on the high side)
    double vbed1=VBED[0];
    double vbed2=VBED[1];
    grndtrck+=sqrt(vbed1*vbed1+vbed2*vbed2)*int_step*REARTH/dbi;
    gndtrkmx=0.001*grndtrck;
    gndtrnmx=NMILES*grndtrck;

    SBII.fill(IPos);
    VBII.fill(IVel);
    ABII.fill(IAccl);
    VBED.fill(vbed);
    FSPB.fill(fspb);
    TDI.fill(tdi);
    TGI.fill(tgi);
    TVD.fill(tvd);
}
