#include <armadillo>
#include <cstring>

#include "matrix/utility.hh"

#include "gyro/gyro_rocket6g.hh"

#include "stochastic.hh"

#include <ctime>



sensor::GyroRocket6G::GyroRocket6G(double emisg[3], double escalg[3], double ebiasg[3])
    :   VECTOR_INIT(EUG    , 3),
        VECTOR_INIT(EWG    , 3),
        VECTOR_INIT(EWALKG , 3),
        VECTOR_INIT(EUNBG  , 3),
        VECTOR_INIT(EMISG  , 3),
        VECTOR_INIT(ESCALG , 3),
        VECTOR_INIT(EBIASG , 3),
        VECTOR_INIT(ITA1, 3),
        VECTOR_INIT(ITA2, 3),
        VECTOR_INIT(BETA, 3) {
    snprintf(name, sizeof(name), "Rocket6G Gyro Sensor Model");
    srand(static_cast<unsigned int>(time(NULL)));
}

void sensor::GyroRocket6G::propagate_error(double int_step) {
    arma::vec3 WBIB = grab_WBIB();
    arma::vec3 FSPB = grab_FSPB();

    //-------------------------------------------------------------------------
    // ARW RRW
    double sig(1.0);
    // std::normal_distribution<double> distribution(0.0, sig);
    double RRW(0.0130848811);  // 0.4422689813  7.6072577e-3
    double ARW(0.2828427125);  // 0.07071067812  7.90569415e-3
    double Freq(200.0);

    for (int i = 0; i < 3; i++) {
        ITA2(i) = gauss(0, 1.0) * RRW * RAD;  // distribution(generator) * RRW * RAD;
        BETA(i) = 0.9999 * BETA(i) + ITA2(i) * int_step;
        ITA1(i) = gauss(0, 1.0) * (ARW * sqrt(Freq) / 60 * (1 / sig)) * RAD;  // distribution(generator) * (ARW * sqrt(Freq) / 60 * (1 / sig)) * RAD;
    }

    // combining all uncertainties
    this->EWBIB = ITA1 + BETA;  // EMSBG + EUG + EWG;

    arma::vec3 tmp = floor((WBIB + EWBIB) / GMSB) * GMSB;
    arma::vec3 tmp2 = floor(((WBIB + EWBIB) - tmp) / GLSB) * GLSB;
    this->WBICB = tmp + tmp2;

    HIGH = floor((WBIB + EWBIB) / GMSB);
    LOW = floor(((WBIB + EWBIB) - tmp) / GLSB);

    return;
}
