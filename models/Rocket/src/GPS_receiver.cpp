#include "GPS_receiver.hh"
#include "trick_utils/math/include/quat_macros.h"
#include "trick_utils/math/include/matrix_macros.h"
#include "trick_utils/math/include/vector_macros.h"
#include "trick_utils/math/include/trick_math_proto.h"
#include "sim_services/include/simtime.h"

#include "global_constants.hh"
#include "utility_header.hh"

#include "GPS_satellites.hh"
#include "Rocket.hh"
#include "Earth.hh"
#include "Ins.hh"

void GPS_Receiver::default_data(){
}

void GPS_Receiver::initialize(Rocket& rkt, GPS_Satellites& sats, INS& i){
    rocket = &rkt;
    gps_sats = &sats;
    ins = &i;
}

void GPS_Receiver::get_quadriga(){
    int i(0);
    int j(0);
    int m(0);

    gdop = LARGE;
    int quad[4] = {0, 0, 0, 0};  // location of quadriga SVs
                                 // in 'ssii_vis[visible_count]

    // conversion to inertial (J2000) coordinates
    double ssii[24][4];
    double sin_incl = sin(gps_sats->inclination);
    double cos_incl = cos(gps_sats->inclination);
    for (i = 0; i < 24; i++) {
        ssii[i][0] =
            gps_sats->radius * (cos(gps_sats->sv_data[i][0]) * cos(gps_sats->sv_data[i][1]) -
                   sin(gps_sats->sv_data[i][0]) * sin(gps_sats->sv_data[i][1]) * cos_incl);
        ssii[i][1] =
            gps_sats->radius * (sin(gps_sats->sv_data[i][0]) * cos(gps_sats->sv_data[i][1]) +
                   cos(gps_sats->sv_data[i][0]) * sin(gps_sats->sv_data[i][1]) * cos_incl);
        ssii[i][2] = gps_sats->radius * sin(gps_sats->sv_data[i][1]) * sin_incl;
        ssii[i][3] = 0;
        // last entry is a flag with the code:
        // =0:not visible; >0:visible (not int but double!), where the
        // number
        // is the SV slot# (1,2,3,...,24)
    }

    // determining visible satellites
    Matrix SSII(3, 1);
    Matrix SBII(3, 1);
    int visible_count(0);

    SBII.build_vec3(rocket->IPos[0], rocket->IPos[1], rocket->IPos[2]);

    for (i = 0; i < 24; i++) {
        SSII[0] = ssii[i][0];
        SSII[1] = ssii[i][1];
        SSII[2] = ssii[i][2];

        // SV to user angle with vertex at Earth center
        double delta = angle(SSII, SBII);

        // grazing angle of SV beam with vertex at Earth center
        double epsilon = acos((Earth::radius + del_rearth) / gps_sats->radius);

        // min radius of user to have clear line-of-site to SV
        double rmin = (Earth::radius + del_rearth) / cos(delta - epsilon);

        if (delta < epsilon) {
            // SV is in the visibility cone
            ssii[i][3] = i + 1;
            visible_count++;
        } else {
            // user is outside the visibility cone but high enough to have
            // LOS to the SV
            // (rmin can go negative if delta-epsilon>90deg; this can happen
            // when the user is opposite (or nearly opposite) of the SV;
            // this is always a no-visibility case)
            double dbi = SBII.absolute();
            if (rmin > 0 && rmin < dbi) {
                ssii[i][3] = i + 1;
                visible_count++;
            }
        }
    }
    // re-acquiring GPS if not enough SVs visible (less than 4)
    if (visible_count < 4) {
        gps_acq = false;
    }
    // selecting best 4 SVs if 4 or more are visible
    else {
        gps_acq = true;

        // repackage visible SVs into 'ssii_vis' single-dimensioned array
        // inertial displacement vector elements are stored sequentially in
        // 'ssii_vis[4*visible_count]'
        // 'ssii_vis' has 3 inertial coordinates and SV slot# of all visible
        // SVs
        double *ssii_vis;
        ssii_vis = new double[4 * visible_count];
        int k(0);
        for (i = 0; i < 24; i++) {
            if (ssii[i][3] > 0) {
                *(ssii_vis + k) = ssii[i][0];
                *(ssii_vis + k + 1) = ssii[i][1];
                *(ssii_vis + k + 2) = ssii[i][2];
                *(ssii_vis + k + 3) = ssii[i][3];
                k = k + 4;
            }
        }
        // selecting quadriga (four SVs) with smallest GDOP
        // i1, i2, i3, i4 are the SVs picked by the binomial combination
        int nm3 = visible_count - 3;  // nm3=1
        int nm2 = visible_count - 2;  // nm2=2
        int nm1 = visible_count - 1;  // nm1=3

        for (int i1 = 0; i1 < nm3; i1++) {
            for (int i2 = i1 + 1; i2 < nm2; i2++) {
                for (int i3 = i2 + 1; i3 < nm1; i3++) {
                    for (int i4 = i3 + 1; i4 < visible_count; i4++) {
                        // pullling the quadriga inertial coordinates
                        Matrix SSII1(3, 1);
                        Matrix SSII2(3, 1);
                        Matrix SSII3(3, 1);
                        Matrix SSII4(3, 1);
                        for (m = 0; m < 3; m++) {
                            SSII1[m] = *(ssii_vis + 4 * i1 + m);
                            SSII2[m] = *(ssii_vis + 4 * i2 + m);
                            SSII3[m] = *(ssii_vis + 4 * i3 + m);
                            SSII4[m] = *(ssii_vis + 4 * i4 + m);
                        }
                        // calculating user wrt the SV displacement unit
                        // vectors
                        Matrix UNI1 = (SBII - SSII1).univec3();
                        Matrix UNI2 = (SBII - SSII2).univec3();
                        Matrix UNI3 = (SBII - SSII3).univec3();
                        Matrix UNI4 = (SBII - SSII4).univec3();

                        // building the GPS 'H' matrix
                        Matrix HGPS(4, 4);
                        HGPS.ones();
                        for (m = 0; m < 3; m++) {
                            HGPS.assign_loc(0, m, UNI1[m]);
                            HGPS.assign_loc(1, m, UNI2[m]);
                            HGPS.assign_loc(2, m, UNI3[m]);
                            HGPS.assign_loc(3, m, UNI4[m]);
                        }
                        // calculating GDOP
                        Matrix COV(4, 4);
                        COV = (HGPS * HGPS.trans()).inverse();
                        double gdop_local =
                            sqrt(COV.get_loc(0, 0) + COV.get_loc(1, 1) +
                                 COV.get_loc(2, 2) + COV.get_loc(3, 3));

                        // save slot # of quadriga SVs if GDOP has decreased
                        if (gdop_local < gdop) {
                            gdop = gdop_local;
                            quad[0] = i1;
                            quad[1] = i2;
                            quad[2] = i3;
                            quad[3] = i4;
                        }
                    }
                }
            }
        }  // end of picking quadriga amongst visible SVs

        // extracting "best" quadriga from visible SVs
        // and storing inertial coordinates of the four SVs and their slot#
        // in ssii_quad[16]
        for (m = 0; m < 4; m++) {
            for (int n = 0; n < 4; n++) {
                *(ssii_quad + 4 * m + n) = *(ssii_vis + 4 * quad[m] + n);
            }
        }
        delete[] ssii_vis;

        // calculating inertial velocity of quadriga SVs
        // getting slot# of quadriga
        //
        int islot[i];
        for (i = 0; i < 4; i++) {
            slot[i] = *(ssii_quad + 4 * i + 3);
            // casting into an int
            islot[i] = (int)slot[i];
        }
        // storing inertial velocities of the four SVs in vsii_quad[12]
        double sin_incl = sin(gps_sats->inclination);
        double cos_incl = cos(gps_sats->inclination);
        double vel = gps_sats->radius * gps_sats->angular_velocity;
        for (m = 0; m < 4; m++) {
            int ii = islot[m] - 1;  // reminder: slot#=1,2,3...,24
            *(vsii_quad + 3 * m + 0) =
                vel *
                (-sin(gps_sats->sv_data[ii][1]) * cos(gps_sats->sv_data[ii][0]) -
                 cos(gps_sats->sv_data[ii][1]) * sin(gps_sats->sv_data[ii][0]) * cos_incl);
            *(vsii_quad + 3 * m + 1) =
                vel *
                (-sin(gps_sats->sv_data[ii][1]) * sin(gps_sats->sv_data[ii][0]) +
                 cos(gps_sats->sv_data[ii][1]) * cos(gps_sats->sv_data[ii][0]) * cos_incl);
            *(vsii_quad + 3 * m + 2) =
                vel * (cos(gps_sats->sv_data[ii][1]) * sin_incl);
        }
    }  // end of picking quadriga from 4 or more visible SVs
}
