#include "cad/env/atmosphere_weatherdeck.hh"

#include <cstring>

cad::Atmosphere_weatherdeck::Atmosphere_weatherdeck(char* filepath) {
    strcpy(name, "Atmosphere Weather Deck");
    read_tables(filepath, weathertable);

    altitude = 0;

    update_values();
}

cad::Atmosphere_weatherdeck::~Atmosphere_weatherdeck() {
}

void cad::Atmosphere_weatherdeck::set_altitude(double altitude_in_meter) {
    altitude = altitude_in_meter;

    update_values();
}

int cad::Atmosphere_weatherdeck::update_values() {
    density  = weathertable.look_up("density", altitude);
    pressure = weathertable.look_up("pressure", altitude);
    tempk    = weathertable.look_up("temperature", altitude) + 273.16;

    vsound = sqrt(1.4 * RGAS * tempk);
}

