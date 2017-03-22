#execfile("Modified_data/realtime.py")
#execfile("Modified_data/rocket.dr")
execfile("Modified_data/test.dr")
# execfile("Modified_data/wangwang.dr")
# trick.exec_set_enable_freeze(True)
# trick.exec_set_freeze_command(True)
# trick.sim_control_panel_set_enabled(True)
##########################################################
new_connection = trick.MSSocket()
trick_master_slave.slave.set_connection_type(new_connection)
trick_master_slave.slave.sync_error_terminate = 1

#############################################################
#Realtime setting
# trick.exec_set_thread_cpu_affinity(0,0)
# trick.exec_set_thread_priority(0,1)
# trick.frame_log_on()
# trick.real_time_enable()
# trick.exec_set_software_frame(0.01)
# trick.itimer_enable()
##############################################################

#INS
"""
frax_algnmnt = 0
fc.ins.set_non_ideal(frax_algnmnt)
"""
fc.ins.set_ideal()

fc.rcs_fc.disable_rcs();        #'int' Attitude control, =|rcs_type||rcs_mode|, see table  module rcs
fc.rcs_fc.set_rcs_tau(1)             #Slope of the switching function - sec  module rcs
fc.rcs_fc.set_thtbdcomx(0)           #Pitch angle command - deg  module rcs
fc.rcs_fc.set_psibdcomx(-85)         #Yaw angle command - deg  module rcs
#Guidance
alphacomx = 0   #Alpha command - deg  module guidance
betacomx = 0    #Beta command - deg  module guidance
fc.guidance.set_degree(alphacomx, betacomx)
######################################################################################################

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
#############################################################

trick.stop(880)
