#include "cad/env/wind_tabular.hh"

#include <cstring>

cad::Wind_Tabular::Wind_Tabular(char* filepath, double twind, double vertical_wind)
    :   Wind(twind, vertical_wind)
{
    strcpy(name, "Tabular Wind");

    altitude = 0;

    read_tables(filepath, weathertable);

    vwind = 0;
    psiwdx = 0;
}

cad::Wind_Tabular::~Wind_Tabular(){
}

void cad::Wind_Tabular::set_altitude(double altitude_in_meter){
    altitude = altitude_in_meter;

    vwind = weathertable.look_up("speed", altitude);
    psiwdx = weathertable.look_up("direction", altitude);
}
