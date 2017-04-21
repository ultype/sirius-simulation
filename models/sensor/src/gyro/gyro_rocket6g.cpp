#include <armadillo>
#include <cstring>

#include "matrix/utility.hh"

#include "gyro/gyro_rocket6g.hh"

#include "stochastic.hh"

#include <ctime>

sensor::GyroRocket6G::GyroRocket6G(double emisg[3], double escalg[3], double ebiasg[3], Newton &newt, _Euler_ &eul, Kinematics &kine)
    :   newton(&newt), euler(&eul), kinematics(&kine),
        VECTOR_INIT(EUG    , 3),
        VECTOR_INIT(EWG    , 3),
        VECTOR_INIT(EWALKG , 3),
        VECTOR_INIT(EUNBG  , 3),
        VECTOR_INIT(EMISG  , 3),
        VECTOR_INIT(ESCALG , 3),
        VECTOR_INIT(EBIASG , 3)
{
    strcpy(name, "Rocket6G Gyro Sensor Model");

    EUG.zeros();
    EWG.zeros();
    EWALKG.zeros();
    EUNBG.zeros();
    srand( (unsigned)time(NULL) );
    for(int i = 0;i < 3;i++){
    
        EMISG(i)  = gauss(0, 1.1e-4);
        ESCALG(i) = gauss(0, 2.e-5);
        EBIASG(i) = gauss(0, 1.e-6);
   
    }
    // EMISG  = arma::vec3(emisg);
    // ESCALG = arma::vec3(escalg);
    // EBIASG = arma::vec3(ebiasg);
}

void sensor::GyroRocket6G::propagate_error(double int_step){
    arma::vec3 WBIB = euler->get_WBIB();
    arma::vec3 FSPB = newton->get_FSPB();
    //-------------------------------------------------------------------------
    // computing cluster misalignment error
    arma::mat33 EGB = diagmat(ESCALG) + skew_sym(EMISG);

    arma::vec3 EMISCG = EGB * WBIB;
    arma::vec3 EMSBG = EBIASG + EMISCG;

    // computing gyro spin axis sensitivity (mass unbalance)
    EUG(0) = EUNBG(0) * FSPB(0);
    EUG(1) = EUNBG(1) * FSPB(1);
    EUG(2) = EUNBG(2) * FSPB(2);

    // computing random walk error
    EWG = EWALKG * (1. / sqrt(int_step));

    // combining all uncertainties
    this->EWBIB = EMSBG + EUG + EWG;

    this->WBICB = WBIB + EWBIB;

    return;
}
