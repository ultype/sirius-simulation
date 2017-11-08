# execfile("Modified_data/test.dr")
execfile("Modified_data/gps.dr")
##########################################################
new_connection = trick.MSSocket()
trick_master_slave.slave.set_connection_type(new_connection)
trick_master_slave.slave.sync_error_terminate = 1

######################set ins start time################################
Year = 2017
DOY = 81
Hour = 2
Min = 0
Sec = 0
fc.time.load_start_time(Year, DOY, Hour, Min, Sec)
#############################################################################

##############################INS############################################
lonx       = 120.893501 # Vehicle longitude - deg  module newton
latx       = 22.138917  # Vehicle latitude  - deg  module newton
alt        = 5        # Vehicle altitude  - m  module newton
fc.ins.load_location(lonx, latx, alt)

phibdx = 0.0      # Rolling  angle of veh wrt geod coord - deg  module kinematics
thtbdx = 90.0 # Pitching angle of veh wrt geod coord - deg  module kinematics
psibdx = 90.0     # Yawing   angle of veh wrt geod coord - deg  module kinematics
fc.ins.load_angle(psibdx, phibdx, thtbdx)

alpha0x    = 0    #Initial angle-of-attack   - deg  module newton
beta0x     = 0    #Initial sideslip angle    - deg  module newton
dvbe       = 0    #Vehicle geographic speed  - m/s  module newton
fc.ins.load_geodetic_velocity(alpha0x, beta0x, dvbe)


fc.ins.set_ideal()
# fc.ins.set_non_ideal()

gpsupdate  = 0
fc.ins.set_gps_correction(gpsupdate);
##########################################################################################

#######################################GPS################################################
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
###############################################################################################

##############################Control variable#################################################
##############################################S2###############################################
S2_mdot = 31.3974
S2_fmass0 = 3139.74
S2_xcg_1 = 4.7888
S2_xcg_0 = 6.4138
S2_isp = 274.8
S2_moi_roll_0     = 180.39
S2_moi_roll_1     = 114.46
S2_moi_pitch_0    = 15883.31
S2_moi_pitch_1    = 7353.05
S2_moi_yaw_0    = 15881.97
S2_moi_yaw_1    = 7351.71
S2_kpp = 3.0
S2_kpi = 0.08
S2_kpd = 0.01
S2_kppp = 6.25
S2_pN = 1000.0
S2_krp = 3.0
S2_kri = 0.33
S2_krd = 0.015
S2_krpp = 9.375
S2_rN = 1000.0
S2_kyp = 3.0
S2_kyi = 0.08
S2_kyd = 0.01
S2_kypp = 7.5
S2_yN = 1000.0
S2_kaoap = 2.2
S2_kaoai = 1.3
S2_kaoad = 0.01
S2_kaoapp = 9.375
S2_aoaN = 1000.0
S2_rollcmd = 0.0
S2_pitchcmd = -7.0
S2_yawcmd = 0.0
S2_aoacmd = 0.0
################################################################################################
#########################################S3#####################################################
S3_mdot = 3.4931
S3_fmass0 = 379.31
S3_xcg_1 = 2.5371
S3_xcg_0 = 2.5808
S3_isp = 290
S3_moi_roll_0     = 26.25
S3_moi_roll_1     = 25.97
S3_moi_pitch_0    = 181.69
S3_moi_pitch_1    = 181.68
S3_moi_yaw_0    = 182.27
S3_moi_yaw_1    = 180.58
S3_kpp = 3.0
S3_kpi = 0.08
S3_kpd = 0.01
S3_kppp = 6.25
S3_pN = 1000.0
S3_krp = 3.0
S3_kri = 0.33
S3_krd = 0.015
S3_krpp = 9.375
S3_rN = 1000.0
S3_kyp = 3.0
S3_kyi = 0.08
S3_kyd = 0.01
S3_kypp = 7.5
S3_yN = 1000.0
S3_kaoap = 2.2
S3_kaoai = 1.3
S3_kaoad = 0.01
S3_kaoapp = 9.375
S3_aoaN = 1000.0
S3_rollcmd = 0.0
S3_pitchcmd = 0.0
S3_yawcmd = 0.0
S3_aoacmd = -29.0
#################################################################################################
fc.control.set_controller_var(S2_mdot, S2_fmass0, S2_xcg_1, S2_xcg_0, S2_isp)
fc.control.set_IBBB0(S2_moi_roll_0, S2_moi_pitch_0, S2_moi_yaw_0)
fc.control.set_IBBB1(S2_moi_roll_1, S2_moi_pitch_1, S2_moi_yaw_1)
# fc.control.set_kpp(S2_kpp)
# fc.control.set_kpi(S2_kpi)
# fc.control.set_kppp(S2_kppp)
# fc.control.set_krp(S2_krp)
# fc.control.set_kri(S2_kri)
# fc.control.set_krpp(S2_krpp)
# fc.control.set_kyp(S2_kyp)
# fc.control.set_kyi(S2_kyi)
# fc.control.set_kypp(S2_kypp)
# fc.control.set_kaoap(S2_kaoap)
# fc.control.set_kaoai(S2_kaoai)
# fc.control.set_kaoad(S2_kaoad)
# fc.control.set_kaoapp(S2_kaoapp)
fc.control.set_attcmd(S2_rollcmd, S2_pitchcmd, S2_yawcmd)
fc.control.set_aoacmd(S2_aoacmd)
fc.control.get_control_gain(S2_kpp, S2_kpi, S2_kpd, S2_kppp, S2_pN, S2_krp, S2_kri, S2_krd, S2_krpp, S2_rN, S2_kyp, S2_kyi, S2_kyd, S2_kypp, S2_yN, S2_kaoap, S2_kaoai, S2_kaoad, S2_kaoapp, S2_aoaN)
######################################################################################################
#########################################################Event:liftoff
liftoff = trick.new_event("liftoff")
liftoff.set_cycle(0.001)
liftoff.condition(0, "trick.exec_get_sim_time() == 0.001")
liftoff.action(0, "fc.ins.set_liftoff(1)")
trick.add_event(liftoff)
liftoff.activate()
######################################################### Event0:control_on
control_rcs_on = trick.new_event("control_rcs_on")
control_rcs_on.set_cycle(0.001)
control_rcs_on.condition(0, "trick.exec_get_sim_time() == 0.001")
control_rcs_on.action(0, "fc.control.set_S2_ROLL_CONTROL()")
trick.add_event(control_rcs_on)
control_rcs_on.activate()
######################################################### Event0:control_on
control_on = trick.new_event("control_on")
control_on.set_cycle(0.001)
control_on.condition(0, "trick.exec_get_sim_time() == 5.001")
control_on.action(0, "fc.control.set_S2_PITCH_DOWN()")
trick.add_event(control_on)
control_on.activate()
######################################################### Event0:control_on
control_on2 = trick.new_event("control_on2")
control_on2.set_cycle(0.001)
control_on2.condition(0, "trick.exec_get_sim_time() == 15.001")
control_on2.action(0, "fc.control.set_attcmd(S2_rollcmd, -9.0, S2_yawcmd)")
trick.add_event(control_on2)
control_on2.activate()
######################################################### Event0:AOA_on
aoac_on = trick.new_event("aoac_on")
aoac_on.set_cycle(0.001)
aoac_on.condition(0, "trick.exec_get_sim_time() == 23.001")
aoac_on.action(0, "fc.control.set_S2_AOA()")
trick.add_event(aoac_on)
aoac_on.activate()
######################################################## Event0:control_off
control_off = trick.new_event("control_off")
control_off.set_cycle(0.001)
control_off.condition(0, "trick.exec_get_sim_time() == 200.001")
control_off.action(0, "fc.control.set_NO_CONTROL()")
trick.add_event(control_off)
# control_off.activate()
######################################################### Event0:S3 control
s3_control_on = trick.new_event("s3_control_on")
s3_control_on.set_cycle(0.001)
s3_control_on.condition(0, "trick.exec_get_sim_time() == 100.001")
s3_control_on.action(0, "fc.control.set_controller_var(S3_mdot, S3_fmass0, S3_xcg_1, S3_xcg_0, S3_isp)")
s3_control_on.action(1, "fc.control.set_IBBB0(S3_moi_roll_0, S3_moi_pitch_0, S3_moi_yaw_0)")
s3_control_on.action(2, "fc.control.set_IBBB1(S3_moi_roll_1, S3_moi_pitch_1, S3_moi_yaw_1)")
s3_control_on.action(3, "fc.control.get_control_gain(S3_kpp, S3_kpi, S3_kpd, S3_kppp, S3_pN, S3_krp, S3_kri, S3_krd, S3_krpp, S3_rN, S3_kyp, S3_kyi, S3_kyd, S3_kypp, S3_yN, S3_kaoap, S3_kaoai, S3_kaoad, S3_kaoapp, S3_aoaN)")
s3_control_on.action(4, "fc.control.set_attcmd(S3_rollcmd, S3_pitchcmd, S3_yawcmd)")
s3_control_on.action(5, "fc.control.set_S3_AOA()")
s3_control_on.action(6, "fc.control.set_ierror_zero()")
s3_control_on.action(7, "fc.control.set_aoacmd(S3_aoacmd)")
trick.add_event(s3_control_on)
s3_control_on.activate()
###############################################################Event1:Stage 2 ignition
# speration_1 = trick.new_event("speration_1")
# speration_1.set_cycle(0.001)
# speration_1.condition(0, "trick.exec_get_sim_time() == 281.001")
# speration_1.action(0, "fc.rcs_fc.set_mode(fc.rcs_fc.INCIDENCE_AND_ROLL_ANGLE_CONTROL)")
# trick.add_event(speration_1)
# speration_1.activate()
##############################################################
#Event3:Stage 3 ignition
# speration_3=trick.new_event("speration_3")
# speration_3.set_cycle(0.001)
# speration_3.condition(0, "rkt.newton.get_thtvdx() < 3.728")
# speration_3.action(0, "fc.rcs_fc.set_mode(fc.rcs_fc.INCIDENCE_AND_ROLL_ANGLE_CONTROL)")
############################################################

trick.stop(200)
