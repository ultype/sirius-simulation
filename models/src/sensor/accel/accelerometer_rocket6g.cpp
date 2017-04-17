#include "sensor/accel/accelerometer_rocket6g.hh"
#include "math/stochastic.hh"
#include "math/matrix/utility.hh"

sensor::AccelerometerRocket6G::AccelerometerRocket6G(double emisa[3], double escala[3], double ebiasa[3], Newton &newt)
    :   newton(&newt),
        VECTOR_INIT(EWALKA, 3),
        VECTOR_INIT(EMISA, 3),
        VECTOR_INIT(ESCALA, 3),
        VECTOR_INIT(EBIASA, 3)
{
    strcpy(name, "Rocket6G Non-Ideal Accelerometer Sensor");

    EWALKA.zeros();
    EMISA.zeros();
    ESCALA.zeros();
    EBIASA.zeros();
    // EMISA  = arma::vec3(emisa);
    // ESCALA = arma::vec3(escala);
    // EBIASA = arma::vec3(ebiasa);

    srand( (unsigned)time(NULL) );
    for(int i = 0;i < 3;i++){
    
        EMISA(i)  = gauss(0, 1.1e-4);
        ESCALA(i) = gauss(0, 5.e-4);
        EBIASA(i) = gauss(0, 3.56e-3);
   
    }
}

void sensor::AccelerometerRocket6G::propagate_error(double int_step){
    arma::vec3 FSPB = newton->get_FSPB();

    // accelerometer error (bias,scale factor,misalignment)
    // acceleration measurement with random walk effect
    //-------------------------------------------------------------------------
    // computing accelerometer erros without random walk (done in 'ins()')
    arma::mat33 EAB = diagmat(ESCALA) + skew_sym(EMISA);
    this->EFSPB = EWALKA + EBIASA + EAB * FSPB;
    //-------------------------------------------------------------------------
    this->FSPCB = FSPB + EFSPB;
}
