#execfile("Modified_data/realtime.py")
#execfile("Modified_data/rocket.dr")
execfile("Modified_data/rocket_csv.dr")
trick.exec_set_enable_freeze(True)
trick.exec_set_freeze_command(True)
trick.sim_control_panel_set_enabled(True)

#SLV
rkt.newton.lonx       = 120.893501    #Vehicle longitude - deg  module newton
rkt.newton.latx       = 22.138917    #Vehicle latitude - deg  module newton
rkt.newton.alt        = 100    #Vehicle altitude - m  module newton
rkt.newton.dvbe       = 2    #Vehicle geographic speed - m/s  module newton

rkt.kinematics.phibdx = 0    #Rolling angle of veh wrt geod coord - deg  module kinematics
rkt.kinematics.thtbdx = 90    #Pitching angle of veh wrt geod coord - deg  module kinematics
rkt.kinematics.psibdx = 90    #Yawing angle of veh wrt geod coord - deg  module kinematics
rkt.newton.alpha0x    = 0    #Initial angle-of-attack - deg  module newton
rkt.newton.beta0x     = 0    #Initial sideslip angle - deg  module newton
#environment
rkt.env.mair = 0              #mair =|matmo|mturb|mwind|
#aerodynamics
rkt.aerodynamics.maero = 13      #=11: last stage; =12: 2 stages; =13: 3 stages
rkt.aerodynamics.xcg_ref = 9.632   #Reference cg location from nose - m
rkt.aerodynamics.refa = 2.36       #Reference area for aero coefficients - m^2
rkt.aerodynamics.refd = 1.7334     #Reference length for aero coefficients - m
rkt.aerodynamics.alplimx = 20      #Alpha limiter for vehicle - deg
rkt.aerodynamics.alimitx = 5       #Structural  limiter for vehicle
#propulsion
rkt.propulsion.mprop  = 3   #'int' =0:none; =3 input; =4 LTG control  module propulsion
rkt.propulsion.vmass0 = 13970       #vehicle initial mass
rkt.propulsion.fmass0 = 8888.9      #vehicle initail fuel mass
rkt.propulsion.xcg_0  = 12.032      #vehicle initial xcg
rkt.propulsion.xcg_1  = 7.965       #vehicle final xcg
rkt.propulsion.moi_roll_0 = 2426.8  #vehicle initial moi in roll direction
rkt.propulsion.moi_roll_1 = 914.5   #vehicle final moi in roll direction
rkt.propulsion.moi_trans_0 = 244537.9   #vehicle initial transverse moi
rkt.propulsion.moi_trans_1 = 87392.2    #vehicle final transverse moi
rkt.propulsion.spi = 255.0          #Specific impusle
rkt.propulsion.fuel_flow_rate = 88.89  #fuel flow rate
rkt.propulsion.aexit = 0.258242843 #nozzle exhaust area
rkt.propulsion.payload = 87 #payload mass
#INS
rkt.ins.mins   = 0
#INS Acceleration
rkt.ins.efspb  = [0, 0, 0]
rkt.ins.ewalka = [0, 0, 0]
rkt.ins.emisa  = [0 ,0 ,0]  #gauss(0, 1.1e-4)
rkt.ins.escala = [0, 0, 0]  #gauss(0, 5e-4)
rkt.ins.ebiasa = [0, 0, 0]  #gauss(0, 3.56e-3)

#ins gyrp
rkt.ins.eug    = [0, 0, 0]
rkt.ins.ewg    = [0, 0, 0]
rkt.ins.ewbib  = [0, 0, 0]
rkt.ins.ewalkg = [0, 0, 0]
rkt.ins.eunbg  = [0, 0, 0]
rkt.ins.emisg  = [0, 0, 0]  #gauss(0, 1.1e-4)
rkt.ins.escalg = [0, 0, 0]  #gauss(0, 2.e-5)
rkt.ins.ebiasg = [0, 0, 0]  #gauss(0, 1.e-6)

#ins grav
rkt.ins.egravi = [0, 0, 0]

#GPS
gps_sats.sats.almanac_time = 80000    #Time since almanac epoch at sim start - sec  module gps
gps_sats.sats.sv_data = [
        [5.63, -1.600],  # A-plane, slot #1
        [5.63, 2.115],   #              #2
        [5.63, -2.309],  #              #3
        [5.63, 0.319],   #              #4

        [0.40, 1.063],   # B-plane, slot #5
        [0.40, -1.342],  #              #6
        [0.40, 0.543],   #              #7
        [0.40, 2.874],   #              #8

        [1.45, 1.705],   # C-plane, slot #9
        [1.45, -2.841],  #              #10
        [1.45, -2.321],  #              #11
        [1.45, -0.640],  #              #12

        [2.45, 1.941],   # D-plane, slot #13
        [2.45, -0.147],  #              #14
        [2.45, 1.690],   #              #15
        [2.45, 0.409],   #              #16

        [3.48, -0.571],  # E-plane, slot #17
        [3.48, -2.988],  #              #18
        [3.48, 0.858],   #              #19
        [3.48, 2.705],   #              #20

        [4.59, -0.7180],  # F-plane,slot #21
        [4.59, 2.666],    #             #22
        [4.59, -2.977],   #             #23
        [4.59, -0.2090]   #             #24
    ]
rkt.gpsr.slot = [0, 0, 0, 0];  #/< SV slot#  of quadriga

rkt.gpsr.del_rearth        = 2317000    #Delta to Earth's radius for GPS clear LOS signal reception - m  module gps
rkt.gpsr.gps_acqtime       = 10    #Acquisition time for GPS signal - s  module gps
rkt.gpsr.gps_step          = 0.1    #GPS update interval - s  module gps

rkt.gpsr.ucfreq_noise      = 0.1 #User clock frequency error - m/s MARKOV  module gps
rkt.gpsr.ucbias_error      = 0 #User clock bias error - m GAUSS  module gps

rkt.gpsr.PR_BIAS           = [0, 0, 0, 0] #Pseudo-range bias - m GAUSS  module gps
rkt.gpsr.PR_NOISE          = [0.25, 0.25, 0.25, 0.25] #Pseudo-range noise - m MARKOV  module gps
rkt.gpsr.DR_NOISE          = [0.03, 0.03, 0.03, 0.03] #Delta-range noise - m/s MARKOV  module gps

rkt.gpsr.uctime_cor = 100  #User clock correlation time constant - s=module gps
rkt.gpsr.ppos        = 5  #Init 1sig pos values of state cov matrix - m=module gps
rkt.gpsr.pvel        = 0.2  #Init 1sig vel values of state cov matrix - m/s=module gps
rkt.gpsr.pclockb     = 3  #Init 1sig clock bias error of state cov matrix - m=module gps
rkt.gpsr.pclockf     = 1  #Init 1sig clock freq error of state cov matrix - m/s=module gps
rkt.gpsr.qpos        = 0.1  #1sig pos values of process cov matrix - m=module gps
rkt.gpsr.qvel        = 0.01  #1sig vel values of process cov matrix - m/s=module gps
rkt.gpsr.qclockb     = 0.5  #1sig clock bias error of process cov matrix - m=module gps
rkt.gpsr.qclockf     = 0.1  #1sig clock freq error of process cov matrix - m/s=module gps
rkt.gpsr.rpos        = 1  #1sig pos value of meas cov matrix - m=module gps
rkt.gpsr.rvel        = 0.1  #1sig vel value of meas cov matrix - m/s=module gps
rkt.gpsr.factp       = 0  #Factor to modifiy initial P-matrix P(1+factp)=module gps
rkt.gpsr.factq       = 0  #Factor to modifiy the Q-matrix Q(1+factq)=module gps
rkt.gpsr.factr       = 0  #Factor to modifiy the R-matrix R(1+factr)=module gps

trick.stop(150)
