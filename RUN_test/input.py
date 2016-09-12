
# execfile("Modified_data/realtime.py")
# execfile("Modified_data/GPS_Satellite.dr")

#SLV
rkt.newton.lonx    = 121.1    #Vehicle longitude - deg  module newton
rkt.newton.latx    = 22.68    #Vehicle latitude - deg  module newton
rkt.newton.alt     = 100    #Vehicle altitude - m  module newton
rkt.newton.dvbe    = 1    #Vehicle geographic speed - m/s  module newton

rkt.kinematics.phibdx   = 0    #Rolling angle of veh wrt geod coord - deg  module kinematics
rkt.kinematics.thtbdx   = 90    #Pitching angle of veh wrt geod coord - deg  module kinematics
rkt.kinematics.psibdx   = 90    #Yawing angle of veh wrt geod coord - deg  module kinematics
rkt.newton.alpha0x = 0    #Initial angle-of-attack - deg  module newton
rkt.newton.beta0x   = 0    #Initial sideslip angle - deg  module newton
#environment

#aerodynamics

#propulsion

#INS

#GPS
gps_sats.sats.almanac_time = 80000    #Time since almanac epoch at sim start - sec  module gps

rkt.gpsr.del_rearth   = 2317000    #Delta to Earth's radius for GPS clear LOS signal reception - m  module gps
rkt.gpsr.gps_acqtime  = 10    #Acquisition time for GPS signal - s  module gps
rkt.gpsr.gps_step     = 0.1    #GPS update interval - s  module gps

rkt.gpsr.ucfreq_noise = 0.1 #User clock frequency error - m/s MARKOV  module gps
rkt.gpsr.ucbias_error = 0 #User clock bias error - m GAUSS  module gps

rkt.gpsr.PR_BIAS      = [0, 0, 0, 0] #Pseudo-range bias - m GAUSS  module gps
rkt.gpsr.PR_NOISE     = [0.25, 0.25, 0.25, 0.25] #Pseudo-range noise - m MARKOV  module gps
rkt.gpsr.DR_NOISE     = [0.03, 0.03, 0.03, 0.03] #Delta-range noise - m/s MARKOV  module gps

trick.stop(180)
