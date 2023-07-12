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
file_path = os.path.dirname(os.path.realpath(getsourcefile(lambda:0)))
sys.path.append(os.path.join(file_path, '..'))
import Grace_Pace_Monitor.grace_instantaneous_state_monitor
import Grace_Instantaneous_Policy.grace_instantaneous_policy



class TurnManager:

    def __init__(self):
        #Miscellaneous
        signal(SIGINT, handle_sigint)
        self.__logger = setupLogger(
                    logging.DEBUG, 
                    logging.INFO, 
                    self.__class__.__name__,
                    "./logs/log_" + datetime.now().strftime("%a_%d_%b_%Y_%I_%M_%S_%p"))
        
        config_path = os.path.dirname(os.path.realpath(getsourcefile(lambda:0))) + "/config/config.yaml"
        self.__config_data = loadConfig(config_path)



        #Ros related components for calling the behavior executor
        self.__nh = True
        rospy.init_node(self.__config_data['Ros']['node_name'])
        self.__grace_behav_client = rospy.ServiceProxy(
                                        self.__config_data['Ros']['grace_behavior_service'], 
                                        grace_attn_msgs.srv.GraceBehavior)




        #Instantiate respective critical components
        self.__state_monitor_inst = Grace_Pace_Monitor.grace_instantaneous_state_monitor.InstantaneousStateMonitor(self.__nh)
        self.__state_monitor_prog = None
        self.__policy_instantaneous = Grace_Instantaneous_Policy.grace_instantaneous_policy.InstantaneousPolicy()
        self.__policy_turn = None



    def __updateStates(self):
        self.__state_monitor_inst.updateState()
        # self.__state_monitor_prog.updateState()


    def __applyPolicy(self):
        instantaneous_actions = self.__policy_instantaneous.applyPolicy(self.__state_monitor_inst.getState())
        progressive_actions = None


        return {'inst_act': instantaneous_actions, 'prog_act': progressive_actions}

    def __mergeExec(self, actions):
        self.__logger.info(actions)


    def mainLoop(self):
        '''
        Question:
            Format of template actions (bc)?
            Format of the output action definitions?
            Fusing output action"
        '''
        rate = rospy.Rate(self.__config_data['General']['main_freq'])
        it_cnt = 0
        while True:
            it_cnt = it_cnt + 1
            print('[Iteration %d]' % it_cnt)
            if(it_cnt == 1):
                #Initial state

                #For instantaneous part, initial state is hardcoded, we just reset timestamp
                self.__state_monitor_inst.initializeState()

                #For progressive part,

            else:
                #Update states
                self.__updateStates()

            #Apply policies
            actions = self.__applyPolicy()

            #Merge and execute actions
            self.__mergeExec(actions)

            #Sleep by rate
            rate.sleep()



if __name__ == '__main__':
    grace_tm = TurnManager()
    grace_tm.mainLoop()