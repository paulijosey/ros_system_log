#!/usr/bin/env python

import rospy
import psutil
import csv 

#    ____            _                   _                      _             
#   / ___| _   _ ___| |_ ___ _ __ ___   | |    ___   __ _  __ _(_)_ __   __ _ 
#   \___ \| | | / __| __/ _ \ '_ ` _ \  | |   / _ \ / _` |/ _` | | '_ \ / _` |
#    ___) | |_| \__ \ ||  __/ | | | | | | |__| (_) | (_| | (_| | | | | | (_| |
#   |____/ \__, |___/\__\___|_| |_| |_| |_____\___/ \__, |\__, |_|_| |_|\__, |
#          |___/                                    |___/ |___/         |___/ 
def systemLog():
    # init node with name
    rospy.init_node('system_log_node', anonymous=True)
    # set rate at which we want to log data
    rate = rospy.Rate(1) # 10Hz

    # init processes to log (vins-estimator, opt-flow-manager & pose-graph)
    # TODO: make those ROS variables? 
    proc_vins = None
    proc_pose = None
    proc_optflow = None
    proc_vins_name = 'vins_estimator'
    proc_pose_name = 'pose_graph'
    proc_optflow_name = 'opt_flow_manager_node'

    # since we don't know the PIDs beforehand check names of running processes
    for proc in psutil.process_iter():
        if (proc.name() == proc_vins_name):
            proc_vins = proc
        elif (proc.name() == proc_pose_name):
            proc_pose = proc
        elif (proc.name() == proc_optflow_name):
            proc_optflow = proc

    config = {}
    # load ROS data
    config_file = rospy.get_param('~config_file', 'default_value')
    with open(config_file) as inf:
        line_words = (line.split(':') for line in inf)
        for line in line_words:
            try:
                config.update({line[0]:line[1].strip().strip('"')})
            except:
                rospy.logwarn("could not read: " + line[0])


    results_path = (config['output_path'] + config['platform'] + "/" +  config['algo'] + 
                    "/" + config['platform'] + "_" + config['algo'] + "_" + 
                    config['dataset'])
    #     ____ ______     __  ____       _   _   _                 
    #    / ___/ ___\ \   / / / ___|  ___| |_| |_(_)_ __   __ _ ___ 
    #   | |   \___ \\ \ / /  \___ \ / _ \ __| __| | '_ \ / _` / __|
    #   | |___ ___) |\ V /    ___) |  __/ |_| |_| | | | | (_| \__ \
    #    \____|____/  \_/    |____/ \___|\__|\__|_|_| |_|\__, |___/
    #                                                    |___/     
    cpu_out_file_name = results_path + "/log_cpu.csv"
    mem_out_file_name = results_path + "/log_mem.csv"

    # this is the header line of the outputed .csv file
    fields = [proc_vins_name, proc_pose_name, proc_optflow_name] 

    with open(cpu_out_file_name, 'w') as csvfile: 
        # creating a csv dict writer object 
        writer_cpu = csv.DictWriter(csvfile, fieldnames = fields) 
        # writing headers (field names) 
        writer_cpu.writeheader() 
        csvfile.close()

    with open(mem_out_file_name, 'w') as csvfile: 
        # creating a csv dict writer object 
        writer = csv.DictWriter(csvfile, fieldnames = fields) 
        # writing headers (field names) 
        writer.writeheader() 
        csvfile.close()

    # spin in this rate
    while not rospy.is_shutdown():
        cpu_dict = {proc_vins_name: proc_vins.cpu_percent(),
                    proc_pose_name: proc_pose.cpu_percent(),
                    proc_optflow_name: proc_optflow.cpu_percent()}

        mem_dict = {proc_vins_name: proc_vins.memory_percent(),
                    proc_pose_name: proc_pose.memory_percent(),
                    proc_optflow_name: proc_optflow.memory_percent()}

        # call logging here
        with open(cpu_out_file_name, 'a') as csvfile: 
            # creating a csv dict writer object 
            writer = csv.DictWriter(csvfile, fieldnames = fields) 
            # writing data rows 
            writer.writerow(cpu_dict)
            csvfile.close()

        with open(mem_out_file_name, 'a') as csvfile: 
            # creating a csv dict writer object 
            writer = csv.DictWriter(csvfile, fieldnames = fields) 
            # writing data rows 
            writer.writerow(mem_dict)
            csvfile.close()

        rate.sleep()

#    __  __       _       
#   |  \/  | __ _(_)_ __  
#   | |\/| |/ _` | | '_ \ 
#   | |  | | (_| | | | | |
#   |_|  |_|\__,_|_|_| |_|
if __name__ == '__main__':
    try:
        systemLog()
    except rospy.ROSInterruptException:
        pass