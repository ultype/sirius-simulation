#execfile("Modified_data/realtime.py")
#execfile("Modified_data/rocket.dr")
execfile("Modified_data/tvc.dr")
# execfile("Modified_data/wangwang.dr")
# trick.exec_set_enable_freeze(True)
# trick.exec_set_freeze_command(True)
# trick.sim_control_panel_set_enabled(True)
##########################################################

#############################################################
#Realtime setting
# trick.exec_set_thread_cpu_affinity(0,0)
# trick.exec_set_thread_priority(0,1)
# trick.frame_log_on()
# trick.real_time_enable()
# trick.exec_set_software_frame(0.01)
# trick.itimer_enable()
##############################################################

##############################################################
#Set simulation start time
Year = 2017
DOY = 61
Hour = 18
Min = 30
Sec = 0
rkt.time.load_start_time(Year, DOY, Hour, Min, Sec)


#SLV
lonx       = 120.893501 # Vehicle longitude - deg  module newton
latx       = 22.138917  # Vehicle latitude  - deg  module newton
alt        = 100        # Vehicle altitude  - m  module newton
rkt.newton.load_location(lonx, latx, alt)

phibdx = 0      # Rolling  angle of veh wrt geod coord - deg  module kinematics
thtbdx = 89 # Pitching angle of veh wrt geod coord - deg  module kinematics
psibdx = 89     # Yawing   angle of veh wrt geod coord - deg  module kinematics
rkt.kinematics.load_angle(psibdx, phibdx, thtbdx)

alpha0x    = 0    #Initial angle-of-attack   - deg  module newton
beta0x     = 0    #Initial sideslip angle    - deg  module newton
dvbe       = 0    #Vehicle geographic speed  - m/s  module newton
rkt.newton.load_geodetic_velocity(alpha0x, beta0x, dvbe)

rkt.euler.load_angular_velocity(0, 0, 0)

#environment
#rkt.env.atmosphere_use_weather_deck("auxiliary/weather_table.txt")
rkt.env.atmosphere_use_public()
rkt.env.set_no_wind()
rkt.env.set_no_wind_turbulunce()
#aerodynamics
rkt.aerodynamics.load_aerotable("auxiliary/aero_table_slv3.txt")
rkt.aerodynamics.set_xcg_ref(9.632)   #Reference cg location from nose - m
rkt.aerodynamics.set_refa(2.36)       #Reference area for aero coefficients - m^2
rkt.aerodynamics.set_refd(1.7334)     #Reference length for aero coefficients - m
rkt.aerodynamics.set_alplimx(20)      #Alpha limiter for vehicle - deg
rkt.aerodynamics.set_alimitx(5)       #Structural  limiter for vehicle
#propulsion
rkt.propulsion.set_vmass0(13970)       #vehicle initial mass
rkt.propulsion.set_fmass0(8888.9)      #vehicle initail fuel mass

xcg_0          = 12.032   # vehicle initial xcg
xcg_1          = 7.965    # vehicle final xcg
moi_roll_0     = 2426.8   # vehicle initial moi in roll direction
moi_roll_1     = 914.5    # vehicle final moi in roll direction
moi_trans_0    = 244537.9 # vehicle initial transverse moi
moi_trans_1    = 87392.2  # vehicle final transverse moi
spi            = 255.0    # Specific impusle
fuel_flow_rate = 88.89    # fuel flow rate

rkt.propulsion.get_input_file_var(xcg_0, xcg_1, moi_roll_0, moi_roll_1, moi_trans_0, moi_trans_1, spi, fuel_flow_rate)# get variable for input file
rkt.propulsion.set_aexit(0.258242843) #nozzle exhaust area
rkt.propulsion.set_payload(98) #payload mass
rkt.propulsion.set_input_thrust(xcg_0, xcg_1, moi_roll_0, moi_roll_1, moi_trans_0, moi_trans_1, spi, fuel_flow_rate)

#INS
"""
frax_algnmnt = 0
rkt.ins.set_non_ideal(frax_algnmnt)
"""
rkt.ins.set_ideal()
#INS Accel
# Create a Errorous Accelerometer
"""
EMISA  = [0, 0, 0]      #gauss(0, 1.1e-4)
ESCALA = [0, 0, 0]      #gauss(0, 2.e-5)
EBIASA = [0, 0, 0]      #gauss(0, 1.e-6)
accel = trick.AccelerometerRocket6G(EMISA, ESCALA, EBIASA, rkt.newton);
"""
# Create a Ideal Accelerometer
accel = trick.AccelerometerIdeal(rkt.newton);
rkt.ins.set_accelerometer(accel);

#ins gyro
# Create a Errorous Gyro
"""
EMISG  = [0, 0, 0]      #gauss(0, 1.1e-4)
ESCALG = [0, 0, 0]      #gauss(0, 2.e-5)
EBIASG = [0, 0, 0]      #gauss(0, 1.e-6)
gyro = trick.GyroRocket6G(EMISG, ESCALG, EBIASG, rkt.newton, rkt.euler, rkt.kinematics);
"""
# Create a Ideal Gyro
gyro = trick.GyroIdeal(rkt.euler);
rkt.ins.set_gyro(gyro);

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

gpsr_factp       = 0   #Factor to modifiy initial P-matrix P(1+factp)=module gps
gpsr_pclockb     = 3   #Init 1sig clock bias error of state cov matrix - m=module gps
gpsr_pclockf     = 1   #Init 1sig clock freq error of state cov matrix - m/s=module gps
rkt.gpsr.setup_state_covariance_matrix(gpsr_factp, gpsr_pclockb, gpsr_pclockf)

gpsr_factq       = 0   #Factor to modifiy the Q-matrix Q(1+factq)=module gps
gpsr_qclockb     = 0.5 #1sig clock bias error of process cov matrix - m=module gps
gpsr_qclockf     = 0.1 #1sig clock freq error of process cov matrix - m/s=module gps
rkt.gpsr.setup_error_covariance_matrix(gpsr_factq, gpsr_qclockb, gpsr_qclockf)

gpsr_uctime_cor = 100  #User clock correlation time constant - s=module gps
rkt.gpsr.setup_fundamental_dynamic_matrix(gpsr_uctime_cor)

rkt.gpsr.ppos        = 5  #Init 1sig pos values of state cov matrix - m=module gps
rkt.gpsr.pvel        = 0.2  #Init 1sig vel values of state cov matrix - m/s=module gps
rkt.gpsr.qpos        = 0.1  #1sig pos values of process cov matrix - m=module gps
rkt.gpsr.qvel        = 0.01  #1sig vel values of process cov matrix - m/s=module gps
rkt.gpsr.rpos        = 1  #1sig pos value of meas cov matrix - m=module gps
rkt.gpsr.rvel        = 0.1  #1sig vel value of meas cov matrix - m/s=module gps
rkt.gpsr.factr       = 0  #Factor to modifiy the R-matrix R(1+factr)=module gps
#RCS thruster
rkt.rcs.disable_rcs();        #'int' Attitude control, =|rcs_type||rcs_mode|, see table  module rcs
rkt.rcs.set_roll_mom_max(100)      #RCS rolling moment max value - Nm  module rcs
rkt.rcs.set_pitch_mom_max(200000)  #RCS pitching moment max value - Nm  module rcs
rkt.rcs.set_yaw_mom_max(200000)    #RCS yawing moment max value - Nm  module rcs
dead_zone = 0.1                #Dead zone of Schmitt trigger - deg  module rcs
hysteresis = 0.1               #Hysteresis of Schmitt trigger - deg  module rcs
rkt.rcs.setup_rcs_schmitt_trigger(dead_zone, hysteresis);
rkt.rcs.set_rcs_tau(1)             #Slope of the switching function - sec  module rcs
rkt.rcs.set_thtbdcomx(0)           #Pitch angle command - deg  module rcs
rkt.rcs.set_psibdcomx(90)         #Yaw angle command - deg  module rcs
rkt.rcs.set_rcs_thrust(100)        #rcs thrust - N  module rcs
rkt.rcs.set_rcs_pos(1.66507)       #rcs thruster's postion from nose - m  module rcs
rkt.rcs.set_rocket_r(0.68)         #rocket's radius - m  module rcs
#Guidance
alphacomx = 0   #Alpha command - deg  module guidance
betacomx = 0    #Beta command - deg  module guidance
rkt.guidance.set_degree(alphacomx, betacomx)

#Control
maut = 55 
thtvdcomx = 88.0
delmix = 7.0
drlmix = 7.0
pgam = 0.15
wgam = 0.5
zgam = 0.7
rkt.control.set_thtvdcomx(thtvdcomx)
rkt.control.set_delimx(delmix)
rkt.control.set_drlimx(drlmix)
rkt.control.set_pgam(pgam)
rkt.control.set_wgam(wgam)
rkt.control.set_zgam(zgam)

##TVC
tvclimx = 10
dtvclimx = 200
wntvc = 100
zettvc = 0.7
factgtvc = 0
parm = 19.25
mtvc = rkt.tvc.NO_DYNAMIC_TVC
gtvc = 1.0
rkt.tvc.set_tvclimx(tvclimx)
rkt.tvc.set_dtvclimx(dtvclimx)
rkt.tvc.set_wntvc(wntvc)
rkt.tvc.set_zettvc(zettvc)
rkt.tvc.set_factgtvc(factgtvc)
rkt.tvc.set_parm(parm)
rkt.tvc.set_mtvc(mtvc)
rkt.tvc.set_gtvc(gtvc)
######################################################################################################

# start = trick.new_event("start")
# start.set_cycle(0.001)
# start.condition(0,"trick.exec_get_sim_time() == 180.001")
# start.action(0,"rkt.propulsion.set_input_thrust(xcg_0, xcg_1, moi_roll_0, moi_roll_1, moi_trans_0, moi_trans_1, spi, fuel_flow_rate)")
# trick.add_event(start)
# start.activate()
#Event0:rcs_on
control_on = trick.new_event("control_on")
control_on.set_cycle(0.001)
control_on.condition(0, "trick.exec_get_sim_time() == 5.001")
# control_on.action(0, "rkt.rcs.enable_rcs()")
control_on.action(0, "rkt.control.set_maut(maut)")
trick.add_event(control_on)
control_on.activate()

######################################################################################################

trick.stop(90)
