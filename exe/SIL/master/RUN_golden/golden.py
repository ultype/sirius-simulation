# execfile("../../../public/Modified_data/realtime.py")
# execfile("../../../public/Modified_data/realtime.dr")
# execfile("../../../public/Modified_data/ui.py")
execfile("../../../public/Modified_data/golden.dr")
##########################################################
# new_connection = trick.MSSocket()
# new_slave = trick.SlaveInfo()
# new_slave.set_connection_type(new_connection)
# new_slave.sim_path = os.getenv("WORKSPACE") + "/exe/PIL/slave"
# new_slave.S_main_name = "./S_main_Linux_5.4_x86_64.exe"
# new_slave.run_input_file = "RUN_golden/golden.py"
# new_slave.sync_error_terminate = 1
# trick_master_slave.master.add_slave(new_slave)
# trick_master_slave.master.enable()

##############################################################
#Set simulation start time
Year = 2017
DOY = 81
Hour = 2
Min = 0
Sec = 0
rkt.time.load_start_time(Year, DOY, Hour, Min, Sec)
rkt.env.dm_RNP()

#SLV
lonx       = 120.893501 # Vehicle longitude - deg  module newton
latx       = 22.138917  # Vehicle latitude  - deg  module newton
alt        = 100        # Vehicle altitude  - m  module newton
rkt.newton.load_location(lonx, latx, alt)

phibdx = 0      # Rolling  angle of veh wrt geod coord - deg  module kinematics
thtbdx = 86.635 # Pitching angle of veh wrt geod coord - deg  module kinematics
psibdx = 90     # Yawing   angle of veh wrt geod coord - deg  module kinematics
rkt.kinematics.load_angle(psibdx, phibdx, thtbdx)

alpha0x    = 0    #Initial angle-of-attack   - deg  module newton
beta0x     = 0    #Initial sideslip angle    - deg  module newton
dvbe       = 0    #Vehicle geographic speed  - m/s  module newton
rkt.newton.load_geodetic_velocity(alpha0x, beta0x, dvbe)

rkt.euler.load_angular_velocity(0, 0, 0)

#INS Accel
# Create a Errorous Accelerometer
"""
EMISA  = [0, 0, 0]      #gauss(0, 1.1e-4)
ESCALA = [0, 0, 0]      #gauss(0, 2.e-5)
EBIASA = [0, 0, 0]      #gauss(0, 1.e-6)
rkt.accelerometer = trick.AccelerometerRocket6G(EMISA, ESCALA, EBIASA, rkt.newton);
"""
# Create a Ideal Accelerometer
rkt.accelerometer = trick.AccelerometerIdeal(rkt.newton);

#ins gyro
# Create a Errorous Gyro
"""
EMISG  = [0, 0, 0]      #gauss(0, 1.1e-4)
ESCALG = [0, 0, 0]      #gauss(0, 2.e-5)
EBIASG = [0, 0, 0]      #gauss(0, 1.e-6)
rkt.gyro = trick.GyroRocket6G(EMISG, ESCALG, EBIASG, rkt.newton, rkt.euler, rkt.kinematics);
"""
# Create a Ideal Gyro
rkt.gyro = trick.GyroIdeal(rkt.euler);

#environment
#rkt.env.atmosphere_use_weather_deck("auxiliary/weather_table.txt")
rkt.env.atmosphere_use_public()
rkt.env.set_no_wind()
rkt.env.set_no_wind_turbulunce()
#aerodynamics
rkt.aerodynamics.load_aerotable("../../../auxiliary/aero_table_slv3.txt")
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

#GPS
# gps_sats.sats.almanac_time = 80000    #Time since almanac epoch at sim start - sec  module gps
# gps_sats.sats.sv_data = [
#         [5.63, -1.600],  # A-plane, slot #1
#         [5.63, 2.115],   #              #2
#         [5.63, -2.309],  #              #3
#         [5.63, 0.319],   #              #4

#         [0.40, 1.063],   # B-plane, slot #5
#         [0.40, -1.342],  #              #6
#         [0.40, 0.543],   #              #7
#         [0.40, 2.874],   #              #8

#         [1.45, 1.705],   # C-plane, slot #9
#         [1.45, -2.841],  #              #10
#         [1.45, -2.321],  #              #11
#         [1.45, -0.640],  #              #12

#         [2.45, 1.941],   # D-plane, slot #13
#         [2.45, -0.147],  #              #14
#         [2.45, 1.690],   #              #15
#         [2.45, 0.409],   #              #16

#         [3.48, -0.571],  # E-plane, slot #17
#         [3.48, -2.988],  #              #18
#         [3.48, 0.858],   #              #19
#         [3.48, 2.705],   #              #20

#         [4.59, -0.7180],  # F-plane,slot #21
#         [4.59, 2.666],    #             #22
#         [4.59, -2.977],   #             #23
#         [4.59, -0.2090]   #             #24
#     ]
# rkt.gpsr.slot = [0, 0, 0, 0];  #/< SV slot#  of quadriga

# rkt.gpsr.del_rearth        = 2317000    #Delta to Earth's radius for GPS clear LOS signal reception - m  module gps
# rkt.gpsr.gps_acqtime       = 10    #Acquisition time for GPS signal - s  module gps
# rkt.gpsr.gps_step          = 0.1    #GPS update interval - s  module gps

# rkt.gpsr.ucfreq_noise      = 0.1 #User clock frequency error - m/s MARKOV  module gps
# rkt.gpsr.ucbias_error      = 0 #User clock bias error - m GAUSS  module gps

# rkt.gpsr.PR_BIAS           = [0, 0, 0, 0] #Pseudo-range bias - m GAUSS  module gps
# rkt.gpsr.PR_NOISE          = [0.25, 0.25, 0.25, 0.25] #Pseudo-range noise - m MARKOV  module gps
# rkt.gpsr.DR_NOISE          = [0.03, 0.03, 0.03, 0.03] #Delta-range noise - m/s MARKOV  module gps

# gpsr_factp       = 0   #Factor to modifiy initial P-matrix P(1+factp)=module gps
# gpsr_pclockb     = 3   #Init 1sig clock bias error of state cov matrix - m=module gps
# gpsr_pclockf     = 1   #Init 1sig clock freq error of state cov matrix - m/s=module gps
# rkt.gpsr.setup_state_covariance_matrix(gpsr_factp, gpsr_pclockb, gpsr_pclockf)

# gpsr_factq       = 0   #Factor to modifiy the Q-matrix Q(1+factq)=module gps
# gpsr_qclockb     = 0.5 #1sig clock bias error of process cov matrix - m=module gps
# gpsr_qclockf     = 0.1 #1sig clock freq error of process cov matrix - m/s=module gps
# rkt.gpsr.setup_error_covariance_matrix(gpsr_factq, gpsr_qclockb, gpsr_qclockf)

# gpsr_uctime_cor = 100  #User clock correlation time constant - s=module gps
# rkt.gpsr.setup_fundamental_dynamic_matrix(gpsr_uctime_cor)

# rkt.gpsr.ppos        = 5  #Init 1sig pos values of state cov matrix - m=module gps
# rkt.gpsr.pvel        = 0.2  #Init 1sig vel values of state cov matrix - m/s=module gps
# rkt.gpsr.qpos        = 0.1  #1sig pos values of process cov matrix - m=module gps
# rkt.gpsr.qvel        = 0.01  #1sig vel values of process cov matrix - m/s=module gps
# rkt.gpsr.rpos        = 1  #1sig pos value of meas cov matrix - m=module gps
# rkt.gpsr.rvel        = 0.1  #1sig vel value of meas cov matrix - m/s=module gps
# rkt.gpsr.factr       = 0  #Factor to modifiy the R-matrix R(1+factr)=module gps
#RCS thruster
rkt.rcs.set_roll_mom_max(100)      #RCS rolling moment max value - Nm  module rcs
rkt.rcs.set_pitch_mom_max(200000)  #RCS pitching moment max value - Nm  module rcs
rkt.rcs.set_yaw_mom_max(200000)    #RCS yawing moment max value - Nm  module rcs
dead_zone = 0.1                #Dead zone of Schmitt trigger - deg  module rcs
hysteresis = 0.1               #Hysteresis of Schmitt trigger - deg  module rcs
rkt.rcs.setup_rcs_schmitt_trigger(dead_zone, hysteresis);
rkt.rcs.set_rcs_thrust(100)        #rcs thrust - N  module rcs
rkt.rcs.set_rcs_pos(1.66507)       #rcs thruster's postion from nose - m  module rcs
rkt.rcs.set_rocket_r(0.68)         #rocket's radius - m  module rcs
######################################################################################################

rkt.gps_con.readfile("../../../auxiliary/brdc0810.17n")

start = trick.new_event("start")
start.set_cycle(0.001)
start.condition(0,"trick.exec_get_sim_time() == 180.001")
start.action(0,"rkt.propulsion.set_input_thrust(xcg_0, xcg_1, moi_roll_0, moi_roll_1, moi_trans_0, moi_trans_1, spi, fuel_flow_rate)")
trick.add_event(start)
start.activate()
#Event1:Stage 2 ignition
speration_1 = trick.new_event("speration_1")
speration_1.set_cycle(0.001)
speration_1.condition(0, "trick.exec_get_sim_time() == 281.001")
speration_1.action(0, "rkt.aerodynamics.set_refa(1.13)")
speration_1.action(1, "rkt.propulsion.set_aexit(0)")
speration_1.action(2, "rkt.aerodynamics.set_xcg_ref(3.429)")

speration_1.action(3, "rkt.propulsion.set_vmass0(3893)")
speration_1.action(4, "rkt.propulsion.set_fmass0(2963)")

speration_1.action(5,  "xcg_0          = 5.829 " )
speration_1.action(6,  "xcg_1          = 4.683 " )
speration_1.action(7,  "moi_roll_0     = 615.2 " )
speration_1.action(8,  "moi_roll_1     = 151.0 " )
speration_1.action(9,  "moi_trans_0    = 7407.4" )
speration_1.action(10, "moi_trans_1    = 3752.1" )
speration_1.action(11, "spi            = 290   " )
speration_1.action(12, "fuel_flow_rate = 29.63 " )
speration_1.action(13, "rkt.propulsion.set_input_thrust(xcg_0, xcg_1, moi_roll_0, moi_roll_1, moi_trans_0, moi_trans_1, spi, fuel_flow_rate)")

speration_1.action(14, "speration_2.activate()")
speration_1.action(15, "rkt.aerodynamics.load_aerotable('../../../auxiliary/aero_table_slv2.txt')")
trick.add_event(speration_1)
speration_1.activate()
###############################################################
#Event2:Fairing speration
speration_2=trick.new_event("speration_2")
speration_2.set_cycle(0.001)
speration_2.condition(0, "trick.exec_get_sim_time() == 350.001")
speration_2.action(0, "rkt.propulsion.set_vmass0(3863)")
speration_2.action(1, "speration_3.activate()")
trick.add_event(speration_2)
##############################################################
#Event3:Stage 3 ignition
speration_3=trick.new_event("speration_3")
speration_3.set_cycle(0.001)
speration_3.condition(0, "rkt.newton.get_thtvdx() < 3.728")
speration_3.action(0, "rkt.rcs.set_rcs_thrust(10)")
speration_3.action(1, "rkt.aerodynamics.set_xcg_ref(3.2489)")
speration_3.action(2, "rkt.rcs.set_roll_mom_max(10)")
speration_3.action(3, "rkt.rcs.set_pitch_mom_max(100)")
speration_3.action(4, "rkt.rcs.set_yaw_mom_max(20)")

speration_3.action(5, "rkt.propulsion.set_vmass0(490)")
speration_3.action(6, "rkt.propulsion.set_fmass0(360)")

speration_3.action(7,  "xcg_0          = 1.942" )
speration_3.action(8,  "xcg_1          = 2.02 " )
speration_3.action(9, "moi_roll_0     = 72.7 " )
speration_3.action(10, "moi_roll_1     = 61.9 " )
speration_3.action(11, "moi_trans_0    = 140.7" )
speration_3.action(12, "moi_trans_1    = 88.8 " )
speration_3.action(13, "spi            = 300  " )
speration_3.action(14, "fuel_flow_rate = 3.33 " )
speration_3.action(15, "rkt.propulsion.set_input_thrust(xcg_0, xcg_1, moi_roll_0, moi_roll_1, moi_trans_0, moi_trans_1, spi, fuel_flow_rate)")

speration_3.action(16, "speration_4.activate()")
speration_3.action(17, "rkt.aerodynamics.load_aerotable('../../../auxiliary/aero_table_slv1.txt')")
trick.add_event(speration_3)
#############################################################
#Event4:MECO
speration_4=trick.new_event("speration_4")
speration_4.set_cycle(0.05)
speration_4.condition(0, "rkt.newton.get_dvbi() > 7613.5")
speration_4.action(0, "rkt.propulsion.set_no_thrust()")
speration_4.action(1, "print trick.exec_get_sim_time()")
#speration_4.action(1, "rkt.propulsion.set_vmass0(0)")
trick.add_event(speration_4)
######################################################################################################

trick.stop(880)
