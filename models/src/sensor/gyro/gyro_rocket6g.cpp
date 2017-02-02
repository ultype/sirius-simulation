#include <armadillo>
#include <cstring>

#include "aux/utility_header.hh"
#include "sensor/gyro/gyro_rocket6g.hh"

sensor::GyroRocket6G::GyroRocket6G(double emisg[3], double escalg[3], double ebiasg[3])
    :   VECTOR_INIT(EUG    , 3),
        VECTOR_INIT(EWG    , 3),
        VECTOR_INIT(EWBIB  , 3),
        VECTOR_INIT(EWALKG , 3),
        VECTOR_INIT(EUNBG  , 3),
        VECTOR_INIT(EMISG  , 3),
        VECTOR_INIT(ESCALG , 3),
        VECTOR_INIT(EBIASG , 3)
{
    strcpy(name, "Rocket6G Gyro Sensor Model");

    EUG.zeros();
    EWG.zeros();
    EWBIB.zeros();
    EWALKG.zeros();
    EUNBG.zeros();

    EMISG  = arma::vec3(emisg);
    ESCALG = arma::vec3(escalg);
    EBIASG = arma::vec3(ebiasg);
}

void sensor::GyroRocket6G::propagate_error(double int_step, arma::vec3 WBIB, arma::vec3 FSPB){
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
    EWBIB = EMSBG + EUG + EWG;

    this->WBICB = WBIB + EWBIB;
    return;
}

arma::vec3 sensor::GyroRocket6G::get_computed_WBIB(){
    return WBICB;
}
