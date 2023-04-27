import rospy
import grace_attn_msgs.msg
import hr_msgs.msg 
import hr_msgs.srv
import std_msgs.msg

import sys
sys.path.append("/home/grace_team/HKUST_GRACE/Grace_Project/Grace_DM")

from config.load_config import load_config
from utils.asr_handler import ASR_Full_Sentence
from utils.log_manager import setup_logger
from utils.start_command_listener import start_command_listener
from utils.mainloop_manager import time_window_manager, engagement_estimator
import time
from utils.robot_trigger import action_trigger
from utils.robot_trigger import stop_trigger
from utils.emotion_recognition_handler import Emotion_Recognition_Handeler
# from utils.data_reader import database_reader
from utils.dialog_handler import DIALOG_HANDLR

from threading import Thread

import os

class multithread_action_wrapper(Thread):
    def __init__(self):
        Thread.__init__(self)
    
    def run(self, function_template, req):
        global robot_speaking
        global logger
        global hardware_interrupt
        global performance_end_timestamp
        global distraction_window_time_stamp
        logger.info("Start to pass actions to robot")
        robot_speaking = True
        execution_result = function_template(req)
        logger.info("Execution result %s" % execution_result)
        hardware_interrupt = (execution_result == "interrupted")
        robot_speaking = False
        distraction_window_time_stamp = time.time()
        performance_end_timestamp = time.time()

def ask_for_repeat(error_message):
    res = chatbot.communicate(args.magic_string["repeat"])
    utterance, params = robot_connector.parse_reply_from_chatbot(res=res)
    req = robot_connector.compose_req(command='exec', utterance=utterance, params=params)
    logger.error(f"Repeat due to {error_message}")
    robot_connector.send_request(req)

def gracefully_end(error_message):
    res = chatbot.communicate(args.magic_string["gracefully_end_conversation"])
    utterance, params = robot_connector.parse_reply_from_chatbot(res=res)
    req = robot_connector.compose_req(command='exec', utterance=utterance, params=params)
    robot_connector.send_request(req)
    robot_connector.stop_conversation(error_message=error_message)

    #Kill the DM process on stop
    print('Now killing the process.')
    sys.exit(0) 


def main_loop():
    """ This is one loop happens in the main frame.
    Basically there are 2 stages, listening stage & speaking stage.

    Args:
        asr (_type_): _description_
    """


    global emergency_stop_flag
    global user_speaking_state
    global robot_speaking
    global hardware_interrupt
    # global performance_end_timestamp
    global distraction_window_time_stamp
    global time_repeat

    # Get patient's engagement level
    engagement_state = em.update_engagement_level()
    # Get user's speaking status
    # receive word --> speaking; receive sentence --> not speaking
    user_speaking_state = time_window.check_asr_input()


    #Special handling of the stop message
    if(stop_command.stop_message):
        gracefully_end(error_message="Stare too long at Grace.")




    logger.debug(f"engagnement={engagement_state}, user_speaking={user_speaking_state}, robot_speaking={robot_speaking}")
    # print(engagement_state)
    ## Check if Grace is speaking, then don't do anything except for tracking engagement level

    if robot_speaking:
        if engagement_state == "Agitated":
            # only handle "Agitated" when patient is speaking
            # FINISH: Do something
            # Stop the orientation
            # 1. Pass an emergency stop to Grace. This will Stop the robot when robot finishes speaking. Set a stoping flag
            # exit(0) # to be replaced by Gracefully end.
            # gracefully_end(f"Agitation detected during robot talking, need to end conversation now.\nengagnement={engagement_state}, user_speaking={user_speaking_state}, robot_speaking={robot_speaking}")
            logger.error("Agitation detected during robot talking, need to end conversation when robot finishes")
            emergency_stop_flag = True
            return
        return

    if hardware_interrupt:
        logger.info("Hardware Bardging in detected")
        # IMPORTANT
        # FINISH: Handle hardware interruption
        # After handling hardware interruption

        # The same way as state==3
        # ==============
        sentence_heard = time_window.get_cached_sentences()
        logger.info(f"User interrupt via Hardware. Now handle user input. Sentence heard: {sentence_heard}")

        #We decide to handle bardging by not doing anything
        '''
        # FINISH: communicate with chatbot to get a response sentence
        # default_action = multithread_action_wrapper()
        # default_action.start()
        res = chatbot.communicate(sentence_heard)
        utterance, params = robot_connector.parse_reply_from_chatbot(res=res)
        req = robot_connector.compose_req(command='exec', utterance=utterance, params=params)
        # robot_connector.send_request(req) # single thread response

        # multithread performance trigger
        multithread_action = multithread_action_wrapper()
        multithread_action.run(robot_connector.send_request, req)
        '''
        # ===============

        hardware_interrupt = False
        return 



    # If patient is speaking, we only run emotion and vision analysis. We will wait for chatbot to generate a reply.
    if user_speaking_state == 1 or user_speaking_state == 2:
        #Update two-window time to prevent premaature stopping
        distraction_window_time_stamp = time.time()

        if engagement_state == "Agitated":
            # only handle "Agitated" when patient is speaking
            # logger.info("Currently Agitated. Patient is agitated when he is speaking")
            # FINISH:
            # Stop the orientation
            # 1. Stop the chatbot when patient finish speaking. Set a stoping flag
            emergency_stop_flag = True
            return
        logger.debug("At this time user is speaking, need wait (do nothing) till he finish")
    # When ASR receives a full sentence, and decided to send it to chat bot
    elif user_speaking_state == 3:
        sentence_heard = time_window.get_cached_sentences()
        logger.info(f"User stoped speaking as word stream input timeout. Start to trigger chatbot. \nSentence heard: {sentence_heard}")

        # FINISH: communicate with chatbot to get a response sentence
        # default_action = multithread_action_wrapper()
        # default_action.start()
        res = chatbot.communicate(sentence_heard)
        utterance, params = robot_connector.parse_reply_from_chatbot(res=res)
        req = robot_connector.compose_req(command='exec', utterance=utterance, params=params)
        # robot_connector.send_request(req) # single thread response

        # multithread performance trigger
        multithread_action = multithread_action_wrapper()
        multithread_action.run(robot_connector.send_request, req)

        # repeat counter
        time_repeat = 0



    # If patient is not speaking now, we think of re-engages. This state patient generally don't reply.
    elif user_speaking_state == 0:
        # If input from user for less than 2 seconds, ignore
        # otherwise treat as distraction
        if time.time() - distraction_window_time_stamp < distraction_time:
            return


        # Handle the emergency stop when patient stop speaking
        if emergency_stop_flag:
            # logger.info("Emergency stop due to agitation")
            # FINISH: gracefully stop the robot. here I only log the message
            # currently I write a return here, though it won't actually stop the loop
            gracefully_end(f"Emergency stop due to agitation when he or she is talking (by flag), current_state: \n engagnement={engagement_state}, user_speaking={user_speaking_state}, robot_speaking={robot_speaking}")
            return
        
        # Patient don't answer with in time_window in if-else branch
        # Check engagement level
        elif engagement_state == "Distracted":
            # logger.info("No feedback from patients and patient is distracted, asking robot to repeat")
            # FINISH: ask chatbot to repeat the question once
            # 1. send an artificial message to chatbot: "Can you repeat?" 
            # - args.magic_string["repeat"]
            # 2. await for chatbot's response
            #time.sleep(2)
            time_repeat += 1
            ask_for_repeat(error_message="No feedback from patients and patient is distracted, asking robot to repeat")
            if time_repeat > 1:
                gracefully_end(error_message)(error_message="Repeated but patient don't get engaged: \n engagnement={engagement_state}, user_speaking={user_speaking_state}, robot_speaking={robot_speaking}")
            return
        elif engagement_state == "Agitated":
            # logger.info("Patient didn't answer and is agitated, ask robot to gracefully stop at once")
            # FINISH: ask chatbot to gracefully end
            # 1. send an artificial message to chatbot: "I don't want to talk anymore" 
            #  - args.magic_string["gracefully_end_conversation"]
            # 2. emergency stop
            # emergency_stop_flag = True
            gracefully_end(error_message="Patient didn't answer and is agitated, ask robot to gracefully stop at once. current_state: \n engagnement={engagement_state}, user_speaking={user_speaking_state}, robot_speaking={robot_speaking}")
            return
        elif engagement_state == "Engaged":
            distraction_window_time_stamp = time.time()
            if time.time() - performance_end_timestamp > stare_but_not_talk_timeout:
                gracefully_end(error_message="Stare too long at Grace.")
    else:
        logger.error(f"user speaking state not in 0,1,2,3, is {user_speaking_state}")
        sys.exit(-1)

        
        
        


        



if __name__ == "__main__":

    #Yifan edit:
    main_loop_node = rospy.init_node("main_loop")

    #Yifan note: put an infinite loop here just for testing
    rate = rospy.Rate(10)#30hz

    # load configs
    args = load_config("./config/config_full10.json")

    # setup logger
    logger = setup_logger()

    # listener for start button
    # currently not integrated to the mainloop
    start_command = start_command_listener(args, logger)
    stop_command = stop_trigger(args)

    # listener for ASR
    asr_listener = ASR_Full_Sentence(args, logger)

    # Emotion listener
    emotion_listener = Emotion_Recognition_Handeler(args)

    # time_window
    time_window = time_window_manager(args, time_window=0.5)

    # engagement estimator
    em = engagement_estimator(emotion_listener)

    # initialize the chatbot upon receiving start message
    while not start_command.start:
        rate.sleep()

    chatbot = DIALOG_HANDLR()

    # Connector to robot
    try:
        robot_connector = action_trigger()
    except rospy.exceptions.ROSException as e:
        logger.error("Fail to connect to robot's custom action service \n" + str(e))
        sys.exit(0) # comment this line if want to start mainloop without custom action APIs

    #Speaking state var
    user_speaking_state = False
    # Whether Grace is speaking
    robot_speaking = False
    # Whether to have emergency stop
    emergency_stop_flag = False
    # whether robot is interrupted
    hardware_interrupt = False
    # timestamp of finishing performance
    performance_end_timestamp = 0
    distraction_window_time_stamp = 0
    distraction_time = 2 # in seconds
    stare_but_not_talk_timeout = 15 # in seconds
    time_repeat = 0


    # Trigger the initial greetings
    # TODO:trigger the greeting
    res = chatbot.communicate(args.magic_string["start_conversation"])
    utterance, params = robot_connector.parse_reply_from_chatbot(res=res)
    req = robot_connector.compose_req(command='exec', utterance=utterance, params=params)
    robot_connector.send_request(req)

    # update the performance timestamp
    performance_end_timestamp = time.time()
    distraction_window_time_stamp = time.time()

    # make sure that ctrl-c can work
    while not rospy.is_shutdown():
        main_loop()
        rate.sleep()#Will make sure this loop runs at 30Hz


