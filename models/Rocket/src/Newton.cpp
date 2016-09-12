#include "newton.hh"
#include "trick_utils/math/include/quat_macros.h"
#include "trick_utils/math/include/matrix_macros.h"
#include "trick_utils/math/include/vector_macros.h"
#include "trick_utils/math/include/trick_math_proto.h"
#include "sim_services/include/simtime.h"

void Newton::default_newton(){
}

void Newton::initialize_newton(Kinematics& kine){
    Kinematics = &kine;

    double psibdx=Kinematics->psibdx;
    double thtbdx=Kinematics->thtbdx;
    double phibdx=Kinematics->phibdx;

    Matrix VBEB(3,1);
    Matrix POLAR(3,1);
    Matrix TBD(3,3);
    //Earth's angular velocity skew-symmetric matrix (3x3)
    WEII.assign_loc(0,1,-WEII3);
    WEII.assign_loc(1,0,WEII3);

    //converting geodetic lonx, latx, alt to SBII
    SBII=cad_in_geo84(lonx*RAD,latx*RAD,alt,time);
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
    TBD=mat3tr(psibdx*RAD,thtbdx*RAD,phibdx*RAD);
    //Geodetic velocity
    VBED=~TBD*VBEB;

    //building inertial velocity
    TDI=cad_tdi84(lonx*RAD,latx*RAD,alt,time);
    TGI=cad_tgi84(lonx*RAD,latx*RAD,alt,time);
    VBII=~TDI*VBED+WEII*SBII;
    dvbi=VBII.absolute();

    //calculating geodetic flight path angles (plotting initialization)
    POLAR=VBED.pol_from_cart();
    psivdx=DEG*POLAR.get_loc(1,0);
    thtvdx=DEG*POLAR.get_loc(2,0);

//Update Matrix variables
    for(i=0;i<9;i++){
        weii[i]=WEII.get_loc((int) i/3,i%3);
    }
    for(i=0;i<3;i++){
        IPos[i]=SBII.get_loc(i,0);
    }
    for(i=0;i<3;i++){
        IVel[i]=VBII.get_loc(i,0);
    }
    for(i=0;i<9;i++){
        tbd[i]=TBD.get_loc((int) i/3,i%3);
    }
    for(i=0;i<3;i++){
        vbed[i]=VBED.get_loc(i,0);
    }
    for(i=0;i<9;i++){
        tdi[i]=TDI.get_loc((int) i/3,i%3);
    }
    for(i=0;i<9;i++){
        tgi[i]=TGI.get_loc((int) i/3,i%3);
    }

}

void Newton::newton(double int_step, Kinematics& kine, Environment& Env, Propulsion& Prop, forces& forc){
    Kinematics=&kine;
    Environment=&Env;
    Propulsion=&Prop;
    forces=&forc;

    Matrix GRAVG(3,1);
    Matrix TBI(3,3);
    Matrix FAPB(3,1);
    Matrix FAP(3,1);
    Matrix TBD(3,3);

    SBII.build_vec3(IPos[0],IPos[1],IPos[2]);
    VBII.build_vec3(IVel[0],IVel[1],IVel[2]);
    ABII.build_vec3(IAccl[0],IAccl[1],IAccl[2]);
    FSPB.build_vec3(fspb[0],fspb[1],fspb[2]);
    TGI.build_mat33(tgi[0],tgi[1],tgi[2],tgi[3],tgi[4],tgi[5],tgi[6],tgi[7],tgi[8]);
    GRAVG.build_vec3(Environment->gravg[0],Environment->gravg[1],Environment->gravg[2]);
    TBI.build_mat33(Kinematics->tbi[0],Kinematics->tbi[1],Kinematics->tbi[2],Kinematics->tbi[3],Kinematics->tbi[4]
                    ,Kinematics->tbi[5],Kinematics->tbi[6],Kinematics->tbi[7],Kinematics->tbi[8]);
    FAPB.build_vec3(forces->fapb[0],forces->fapb[1],forces->fapb[2]);
    FAP.build_vec3(forces->FAP[0],forces->FAP[1],forces->FAP[2]);
    TVD.build_mat33(tvd[0],tvd[1],tvd[2],tvd[3],tvd[4],tvd[5],tvd[6],tvd[7],tvd[8]);
    TBD.build_mat33(Kinematics->tbd[0],Kinematics->tbd[1],Kinematics->tbd[2],Kinematics->tbd[3],Kinematics->tbd[4],Kinematics->tbd[5]
                    ,Kinematics->tbd[6],Kinematics->tbd[7],Kinematics->tbd[8]);

    double grav=Environment->grav;
    double vmass=Propulsion->vmass;



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

    cad_geo84_in(lon,lat,alt,SBII,time);
    TDI=cad_tdi84(lon,lat,alt,time);
    TGI=cad_tgi84(lon,lat,alt,time);
    lonx=lon*DEG;
    latx=lat*DEG;
    //geographic velocity in geodetic axes VBED(3x1) and flight path angles
    VBED=TDI*(VBII-WEII*SBII);
    Matrix POLAR=VBED.pol_from_cart();
    dvbe=POLAR[0];
    psivdx=DEG*POLAR[1];
    thtvdx=DEG*POLAR[2];
    //calculate gravity loss
    gravity_loss2=gravity_loss2+grav*sin(thtvdx*RAD)*int_step;

    //T.M. of geographic velocity wrt geodetic coordinates
    TVD=mat2tr(psivdx*RAD,thtvdx*RAD);

    //diagnostics: acceleration achieved
    ayx=FSPB[1]/AGRAV;
    anx=-FSPB[2]/AGRAV;

    //ground track travelled (10% accuracy, usually on the high side)
    double vbed1=VBED[0];
    double vbed2=VBED[1];
    grndtrck+=sqrt(vbed1*vbed1+vbed2*vbed2)*int_step*REARTH/dbi;
    gndtrkmx=0.001*grndtrck;
    gndtrnmx=NMILES*grndtrck;

    //update Martix variables
    for(i=0;i<3;i++){
        IPos[i]=SBII.get_loc(i,0);
    }
    for(i=0;i<3;i++){
        IVel[i]=VBII.get_loc(i,0);
    }
    for(i=0;i<3;i++){
        IAccl[i]=ABII.get_loc(i,0);
    }
    for(i=0;i<3;i++){
        vbed[i]=VBED.get_loc(i,0);
    }
    for(i=0;i<3;i++){
        fspb[i]=FSPB.get_loc(i,0);
    }
    for(i=0;i<9;i++){
        tdi[i]=TDI.get_loc((int) i/3,i%3);
    }
    for(i=0;i<9;i++){
        tgi[i]=TGI.get_loc((int) i/3,i%3);
    }
    for(i=0;i<9;i++){
        tvd[i]=TVD.get_loc((int) i/3,i%3);
    }

}
