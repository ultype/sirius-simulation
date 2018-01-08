trick.real_time_enable()
trick.exec_set_software_frame(0.05) # 20PPS
trick.clock_set_rt_clock_ratio(10)
trick.itimer_enable()

new_connection = trick.MSSocket()
new_slave = trick.SlaveInfo()
new_slave.set_connection_type(new_connection)
new_slave.sim_path = "/home/sonicyang/gitRepo/sirius-simulation/slave"
new_slave.S_main_name = "./S_main_Linux_5.4_x86_64.exe"
new_slave.run_input_file = "RUN_test/test.py"
new_slave.sync_error_terminate = 1
trick_master_slave.master.add_slave(new_slave)
trick_master_slave.master.enable()
