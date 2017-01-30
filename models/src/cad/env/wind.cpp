#include "cad/env/wind.hh"
#include "aux/global_constants.hh"
#include "aux/utility_header.hh"

cad::Wind::Wind(double twind, double vertical_wind)
    :   VECTOR_INIT(VAED, 3),
        VECTOR_INIT(VAEDS, 3),
        VECTOR_INIT(VAEDSD, 3)
{
    this->twind = twind;
    this->vertical_wind_speed = vertical_wind;

    VAED.zeros();
    VAEDS.zeros();
    VAEDSD.zeros();
};

void cad::Wind::propagate_VAED(double int_step){
    //wind components in geodetic coordinates
    arma::vec3 VAED_RAW;
    VAED_RAW(0) = -vwind * cos(psiwdx * RAD);
    VAED_RAW(1) = -vwind * sin(psiwdx * RAD);
    VAED_RAW(2) = this->vertical_wind_speed;

    //smoothing wind by filtering with time constant 'twind' sec
    arma::vec3 VAEDSD_NEW = (VAED_RAW - VAEDS) * (1 / twind);
    VAEDS = integrate(VAEDSD_NEW, VAEDSD, VAEDS, int_step);
    VAEDSD = VAEDSD_NEW;
    VAED = VAEDS;

};

