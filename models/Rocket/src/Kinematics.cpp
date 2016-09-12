#include "kinematics.hh"
#include "trick_utils/math/include/quat_macros.h"
#include "trick_utils/math/include/matrix_macros.h"
#include "trick_utils/math/include/vector_macros.h"
#include "trick_utils/math/include/trick_math_proto.h"
#include "sim_services/include/simtime.h"


void Kinematics::init_kinematics(double time, double int_step, Newton& newt){
    Newton=&newt;

    double lonx=Newton->lonx;
    double latx=Newton->latx;
    double alt=Newton->alt;
    double dvbe=Newton->dvbe;
    Matrix TDI(3,3);

    TBD=mat3tr(psibdx*RAD,thtbdx*RAD,phibdx*RAD);
    TDI=cad_tdi84(lonx*RAD,latx*RAD,alt,time);
    TBI=TBD*TDI;
    //update Matrix variables
    for(i=0;i<9;i++){
        tbd[i]=TBD.get_loc((int) i/3,i%3);
    }
    for(i=0;i<9;i++){
        tbi[i]=TBI.get_loc((int) i/3,i%3);
    }

}

void Kinematics::kinematics(double int_step, Environment& env, Euler& eul, Newton& newt){
    Environment=&env;
    Euler=&eul;
    Newton=&newt;

    Matrix UNIT(3,3); UNIT.identity();
    double cthtbd(0);
    double phip(0);

    double dvba=Environment->dvba;
    double lonx=Newton->lonx;
    double latx=Newton->latx;
    double alt=Newton->alt;

    Matrix WBIB(3,1);
    Matrix VBED(3,1);
    Matrix VAED(3,1);
    Matrix VBII(3,1);

    WBIB.build_vec3(Euler->wbib[0],Euler->wbib[1],Euler->wbib[2]);
    VBED.build_vec3(Newton->vbed[0],Newton->vbed[1],Newton->vbed[2]);
    VAED.build_vec3(Environment->vaed[0],Environment->vaed[1],Environment->vaed[2]);
    VBII.build_vec3(Newton->IVel[0],Newton->IVel[1],Newton->IVel[2]);

    //*integrating direction cosine matrix
    Matrix TBID_NEW=~WBIB.skew_sym()*TBI;
    TBI=integrate(TBID_NEW,TBID,TBI,int_step);
    TBID=TBID_NEW;

    //orthonormalizing TBI
    Matrix EE=UNIT-TBI*~TBI;
    TBI=TBI+EE*TBI*0.5;

    //TBI orthogonality check
    double e1=EE.get_loc(0,0);
    double e2=EE.get_loc(1,1);
    double e3=EE.get_loc(2,2);
    ortho_error=sqrt(e1*e1+e2*e2+e3*e3);

    //Euler angles
    Matrix TDI=cad_tdi84(lonx*RAD,latx*RAD,alt,time);
    TBD=TBI*~TDI;
    double tbd13=TBD.get_loc(0,2);
    double tbd11=TBD.get_loc(0,0);
    double tbd33=TBD.get_loc(2,2);
    double tbd12=TBD.get_loc(0,1);
    double tbd23=TBD.get_loc(1,2);

    //*geodetic Euler angles
    //pitch angle: 'thtbd'
    //note: when |tbd13| >= 1, thtbd = +- pi/2, but cos(thtbd) is
    //      forced to be a small positive number to prevent division by zero
    if(fabs(tbd13)<1){
        thtbd=asin(-tbd13);
        cthtbd=cos(thtbd);
    }
    else{
        thtbd=PI/2*sign(-tbd13);
        cthtbd=EPS;
    }
    //yaw angle: 'psibd'
    double cpsi=tbd11/cthtbd;
    if(fabs(cpsi)>1)
        cpsi=1*sign(cpsi);
    psibd=acos(cpsi)*sign(tbd12);

    //roll angle: 'phibdc'
    double cphi=tbd33/cthtbd;
    if(fabs(cphi)>1)
        cphi=1*sign(cphi);
    phibd=acos(cphi)*sign(tbd23);

    psibdx=DEG*psibd;
    thtbdx=DEG*thtbd;
    phibdx=DEG*phibd;

    //*incidence angles using wind vector VAED in geodetic coord
    Matrix VBAB=TBD*(VBED-VAED);
    double vbab1=VBAB.get_loc(0,0);
    double vbab2=VBAB.get_loc(1,0);
    double vbab3=VBAB.get_loc(2,0);
    double alpha=atan2(vbab3,vbab1);
    double beta=asin(vbab2/dvba);
    alphax=alpha*DEG;
    betax=beta*DEG;

    //incidence angles in load factor plane (aeroballistic)
    double dum=vbab1/dvba;

    if(fabs(dum)>1)
        dum=1*sign(dum);
    double alpp=acos(dum);
    if(vbab2==0&&vbab3==0)
        phip=0.;
    //note: if vbeb2 is <EPS the value phip is forced to be 0 or PI
    //      to prevent oscillations
    else if(fabs(vbab2)<EPS)
        if(vbab3>0) phip=0;
        if(vbab3<0) phip=PI;
    else
        phip=atan2(vbab2,vbab3);
    alppx=alpp*DEG;
    phipx=phip*DEG;

    //*diagnostic: calculating the inertial incidence angles
    Matrix VBIB=TBI*VBII;
    double vbib1=VBIB.get_loc(0,0);
    double vbib2=VBIB.get_loc(1,0);
    double vbib3=VBIB.get_loc(2,0);
    double alphai=atan2(vbib3,vbib1);
    double dvbi=VBIB.absolute();
    double betai=asin(vbib2/dvbi);
    alphaix=alphai*DEG;
    betaix=betai*DEG;

    //update Matrix variables
    for(i=0;i<9;i++){
        tbi[i]=TBI.get_loc((int) i/3,i%3);
    }
    for(i=0;i<9;i++){
        tbid[i]=TBID.get_loc((int) i/3,i%3);
    }
    for(i=0;i<9;i++){
        tbd[i]=TBD.get_loc((int) i/3,i%3);
    }

}
