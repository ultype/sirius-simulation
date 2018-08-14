#include "accel/accelerometer_rocket6g.hh"
#include "stochastic.hh"
#include "matrix/utility.hh"

sensor::AccelerometerRocket6G::AccelerometerRocket6G(double emisa[3], double escala[3], double ebiasa[3])
    :   VECTOR_INIT(EWALKA, 3),
        VECTOR_INIT(EMISA, 3),
        VECTOR_INIT(ESCALA, 3),
        VECTOR_INIT(EBIASA, 3),
        VECTOR_INIT(ITA1, 3),
        VECTOR_INIT(ITA2, 3),
        VECTOR_INIT(BETA, 3) {
    snprintf(name, sizeof(name), "Rocket6G Non-Ideal Accelerometer Sensor");
    srand(static_cast<unsigned int>(time(NULL)));
    EWALKA.zeros();
    EMISA.zeros();
    ESCALA.zeros();
    EBIASA.zeros();
    // EMISA  = arma::vec3(emisa);
    // ESCALA = arma::vec3(escala);
    // EBIASA = arma::vec3(ebiasa);

    // srand((unsigned)time(NULL));
    // for (int i = 0; i < 3; i++) {
    //     EMISA(i)  = gauss(0, 1.1e-4);
    //     ESCALA(i) = gauss(0, 5.e-4);
    //     EBIASA(i) = gauss(0, 3.56e-3);
    // }
}

void sensor::AccelerometerRocket6G::propagate_error(double int_step) {
    arma::vec3 FSPB = grab_FSPB();

    // accelerometer error (bias,scale factor,misalignment)
    // acceleration measurement with random walk effect
    //-------------------------------------------------------------------------
    // computing accelerometer erros without random walk (done in 'ins()')
    // arma::mat33 EAB = diagmat(ESCALA) + skew_sym(EMISA);
    // this->EFSPB = EWALKA + EBIASA + EAB * FSPB;
    // //-------------------------------------------------------------------------
    double sig(1.0);

    double RRW(0.01647856578);  // 0.4422689813  7.6072577e-3
    double ARW(0.01025304833);  // 0.07071067812  7.90569415e-3
    double Freq(200.0);
    // for (int i = 0; i < 3; i++) {
    //     ITA2(i) = gauss(0, sig) * RRW * RAD;
    //     BETA(i) = 0.999 * BETA_old(i) + ITA2(i) * int_step;
    //     ITA1(i) = gauss(0, sig) * (ARW * sqrt(Freq) / 60 * (1 / sig)) * RAD;
    // }
    for (int i = 0; i < 3; i++) {
        ITA2(i) = gauss(0, 1.0) * RRW;
        BETA(i) = 0.9999 * BETA(i) + ITA2(i) * int_step;
        ITA1(i) = gauss(0, 1.0) * (ARW * sqrt(Freq) / 60 * (1 / sig));
    }

    // combining all uncertainties
    this->EFSPB = ITA1 + BETA;  // EMSBG + EUG + EWG;

    // this->FSPCB = FSPB + EFSPB;

    arma::vec3 tmp = floor((FSPB + EFSPB) / AMSB) * AMSB;
    arma::vec3 tmp2 = floor(((FSPB + EFSPB) - tmp) / ALSB) * ALSB;
    this->FSPCB = tmp + tmp2;

    HIGH = floor((FSPB + EFSPB) / AMSB);
    LOW = floor(((FSPB + EFSPB) - tmp) / ALSB);
}
