#general
import yaml
import rospy
import os
import re
import threading
from signal import signal
from signal import SIGINT
import logging
import sys
from datetime import datetime
import time
from inspect import getsourcefile
from os.path import abspath


import dynamic_reconfigure.client
import sensor_msgs.msg
import std_msgs.msg
import hr_msgs.msg
import grace_attn_msgs.msg
import grace_attn_msgs.srv
import hr_msgs.msg
import hr_msgs.cfg
import hr_msgs.srv
import std_msgs


#Load configs
def loadConfig(path):
    #Load configs
    with open(path, "r") as config_file:
        config_data = yaml.load(config_file, Loader=yaml.FullLoader)
        # print("Config file loaded")
    return config_data

#Create Logger
def setupLogger(file_log_level, terminal_log_level, logger_name, log_file_name):
    log_formatter = logging.Formatter('%(asctime)s %(msecs)03d %(name)s %(levelname)s %(funcName)s(%(lineno)d) %(message)s', 
                                  datefmt='%d/%m/%Y %H:%M:%S')

    f = open(log_file_name, "a")
    f.close()
    file_handler = logging.FileHandler(log_file_name)
    file_handler.setFormatter(log_formatter)
    file_handler.setLevel(file_log_level)

    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(log_formatter)
    stream_handler.setLevel(terminal_log_level)

    logger = logging.getLogger(logger_name)
    logger.addHandler(file_handler)
    logger.addHandler(stream_handler)
    logger.setLevel( min(file_log_level,terminal_log_level) )#set to lowest

    return logger

#Respond to exit signal
def handle_sigint(signalnum, frame):
    # terminate
    print('Main interrupted! Exiting.')
    sys.exit()



#Load necessary modules
file_path = abspath(getsourcefile(lambda:0))
sys.path.insert(0,file_path + '../Grace_Pace_Monitor')
sys.path.insert(0,file_path + '../Grace_Instantaneous_Policy')

import Grace_Pace_Monitor
import Grace_Instantaneous_Policy



class TurnManager:

    def __init__(self):
        #Miscellaneous
        signal(SIGINT, handle_sigint)
        self.__logger = setupLogger(
                    logging.DEBUG, 
                    logging.INFO, 
                    self.__class__.__name__,
                    "./logs/log_" + datetime.now().strftime("%a_%d_%b_%Y_%I_%M_%S_%p"))

        config_path = abspath(getsourcefile(lambda:0)) + "./config/config.yaml"
        self.__config_data = loadConfig(config_path)



        #Ros related components for calling the behavior executor
        self.__nh = rospy.init_node(self.__config_data['Ros']['node_name'])
        self.__grace_behav_client = rospy.ServiceProxy(
                                        self.__configs['Ros']['grace_behavior_service'], 
                                        grace_attn_msgs.srv.GraceBehavior)




        #Instantiate respective critical components
        self.__state_monitor_pace = Grace_Pace_Monitor.grace_pace_monitor.PaceMonitor(self.__nh)
        self.__state_monitor_turn = None
        self.__policy_instantaneous = Grace_Instantaneous_Policy.grace_instantaneous_policy.InstantaneousPolicy()
        self.__policy_turn = None




    def __mainLoop(self):
        #Update respective parts of the turn states

        #Instantaneous and Progressive policies make their decisions

        #Merge the decisions (some aspects are overlapping, e.g., bc and dialogue)

        #Call the uniform behavior handle and execute the actions issued

        pass


