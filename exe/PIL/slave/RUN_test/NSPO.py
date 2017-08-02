# execfile("Modified_data/test.dr")
execfile("Modified_data/gps.dr")
##########################################################
new_connection = trick.MSSocket()
trick_master_slave.slave.set_connection_type(new_connection)
trick_master_slave.slave.sync_error_terminate = 1

Year = 2017
DOY = 81
Hour = 2
Min = 0
Sec = 0
fc.time.load_start_time(Year, DOY, Hour, Min, Sec)

#INS
lonx       = 120.893501 # Vehicle longitude - deg  module newton
latx       = 22.138917  # Vehicle latitude  - deg  module newton
alt        = 100        # Vehicle altitude  - m  module newton
fc.ins.load_location(lonx, latx, alt)

phibdx = 0      # Rolling  angle of veh wrt geod coord - deg  module kinematics
thtbdx = 86.615 # Pitching angle of veh wrt geod coord - deg  module kinematics
psibdx = 90     # Yawing   angle of veh wrt geod coord - deg  module kinematics
fc.ins.load_angle(psibdx, phibdx, thtbdx)

alpha0x    = 0    #Initial angle-of-attack   - deg  module newton
beta0x     = 0    #Initial sideslip angle    - deg  module newton
dvbe       = 0    #Vehicle geographic speed  - m/s  module newton
fc.ins.load_geodetic_velocity(alpha0x, beta0x, dvbe)


# fc.ins.set_ideal()
fc.ins.set_non_ideal()

gpsupdate  = 0
fc.ins.set_gps_correction(gpsupdate);

fc.rcs_fc.disable_rcs();        #'int' Attitude control, =|rcs_type||rcs_mode|, see table  module rcs
fc.rcs_fc.set_rcs_tau(1)             #Slope of the switching function - sec  module rcs
fc.rcs_fc.set_thtbdcomx(0)           #Pitch angle command - deg  module rcs
fc.rcs_fc.set_psibdcomx(-85)         #Yaw angle command - deg  module rcs
#Guidance
alphacomx = 0   #Alpha command - deg  module guidance
betacomx = 0    #Beta command - deg  module guidance
fc.guidance.set_degree(alphacomx, betacomx)

### GPS
fc.gps.ucfreq_noise      = 0.1 #User clock frequency error - m/s MARKOV  module gps
fc.gps.ucbias_error      = 0 #User clock bias error - m GAUSS  module gps

fc.gps.PR_BIAS           = [0, 0, 0, 0] #Pseudo-range bias - m GAUSS  module gps
fc.gps.PR_NOISE          = [0.25, 0.25, 0.25, 0.25] #Pseudo-range noise - m MARKOV  module gps
fc.gps.DR_NOISE          = [0.03, 0.03, 0.03, 0.03] #Delta-range noise - m/s MARKOV  module gps

gpsr_factp       = 0   #Factor to modifiy initial P-matrix P(1+factp)=module gps
gpsr_pclockb     = 3   #Init 1sig clock bias error of state cov matrix - m=module gps
gpsr_pclockf     = 1   #Init 1sig clock freq error of state cov matrix - m/s=module gps
fc.gps.setup_state_covariance_matrix(gpsr_factp, gpsr_pclockb, gpsr_pclockf)

gpsr_factq       = 0   #Factor to modifiy the Q-matrix Q(1+factq)=module gps
gpsr_qclockb     = 0.5 #1sig clock bias error of process cov matrix - m=module gps
gpsr_qclockf     = 0.1 #1sig clock freq error of process cov matrix - m/s=module gps
fc.gps.setup_error_covariance_matrix(gpsr_factq, gpsr_qclockb, gpsr_qclockf)

gpsr_uctime_cor = 100  #User clock correlation time constant - s=module gps
fc.gps.setup_fundamental_dynamic_matrix(gpsr_uctime_cor)

fc.gps.ppos        = 5  #Init 1sig pos values of state cov matrix - m=module gps
fc.gps.pvel        = 0.2  #Init 1sig vel values of state cov matrix - m/s=module gps
fc.gps.qpos        = 0.1  #1sig pos values of process cov matrix - m=module gps
fc.gps.qvel        = 0.01  #1sig vel values of process cov matrix - m/s=module gps
fc.gps.rpos        = 1  #1sig pos value of meas cov matrix - m=module gps
fc.gps.rvel        = 0.1  #1sig vel value of meas cov matrix - m/s=module gps
fc.gps.factr       = 0  #Factor to modifiy the R-matrix R(1+factr)=module gps
######################################################################################################
#Event:liftoff
liftoff = trick.new_event("liftoff")
liftoff.set_cycle(0.001)
liftoff.condition(0, "trick.exec_get_sim_time() == 180.002")
liftoff.action(0, "fc.ins.set_liftoff(1)")
trick.add_event(liftoff)
liftoff.activate()
#Event0:rcs_on
rcs_on = trick.new_event("rcs_on")
rcs_on.set_cycle(0.001)
rcs_on.condition(0, "trick.exec_get_sim_time() == 185.001")
rcs_on.action(0, "fc.rcs_fc.enable_rcs()")
trick.add_event(rcs_on)
rcs_on.activate()
#Event1:Stage 2 ignition
speration_1 = trick.new_event("speration_1")
speration_1.set_cycle(0.001)
speration_1.condition(0, "trick.exec_get_sim_time() == 281.001")
speration_1.action(0, "fc.rcs_fc.set_mode(fc.rcs_fc.INCIDENCE_AND_ROLL_ANGLE_CONTROL)")
trick.add_event(speration_1)
speration_1.activate()
##############################################################
#Event3:Stage 3 ignition
speration_3=trick.new_event("speration_3")
speration_3.set_cycle(0.001)
speration_3.condition(0, "rkt.newton.get_thtvdx() < 3.728")
speration_3.action(0, "fc.rcs_fc.set_mode(fc.rcs_fc.INCIDENCE_AND_ROLL_ANGLE_CONTROL)")
############################################################

trick.stop(880)
