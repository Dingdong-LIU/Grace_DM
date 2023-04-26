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
from utils.emotion_recognition_handler import Emotion_Recognition_Handeler

from threading import Thread

class multithread_action_wrapper(Thread):
    def __init__(self):
        Thread.__init__(self)
    
    def run(self):
        global robot_speaking
        global logger
        global hardware_interrupt
        logger.info("Start to pass actions to robot")
        robot_speaking = True
        execution_result = robot_connector.test_send_request()
        logger.info("Execution result %s" % execution_result)
        hardware_interrupt = (execution_result == "interrupted")
        robot_speaking = False


def main_loop():
    """ This is one loop happens in the main frame.
    Basically there are 2 stages, listening stage & speaking stage.

    Args:
        asr (_type_): _description_
    """

    # # print(asr_listener.asr_full_sentence) # For debug Only
    # logger.info("enter main loop")

    global emergency_stop_flag
    global user_speaking_state
    global robot_speaking
    global hardware_interrupt


    ## Check if Grace is speaking, then don't do anything except for tracking engagement level
    engagement_state = em.update_engagement_level()

    user_speaking_state = time_window.check_asr_input()
    # receive word --> speaking; receive sentence --> not speaking



    logger.info(f"engagnement={engagement_state}, user_speaking={user_speaking_state}, robot_speaking={robot_speaking}")

    # print(engagement_state)
    if robot_speaking:
        if engagement_state == "Agitated":
            # only handle "Agitated" when patient is speaking
            logger.info("Agitation detected during robot talking, need to end conversation now")
            # TODO: Do something
            # Stop the orientation
            # 1. Pass an emergency stop to Grace. 
            # 2. Stop the chatbot when patient finish speaking. Set a stoping flag
            # exit(0) # to be replaced by Gracefully end.
            return
        if user_speaking_state == 1 or 2:
            logger.info("Bardging in detected via software")
        return

    if hardware_interrupt:
        logger.info("Hardware Bardging in detected")

        # After handling hardware interruption
        hardware_interrupt = False
        return 

    


    # If patient is speaking, we only run emotion and vision analysis. We will wait for chatbot to generate a reply.
    if user_speaking_state == 1 or user_speaking_state == 2:
        if engagement_state == "Agitated":
            # only handle "Agitated" when patient is speaking
            logger.info("Currently Agitated. Patient is agitated when he is speaking")
            # TODO: Do something
            # Stop the orientation
            # 1. Pass an emergency stop to Grace. 
            # 2. Stop the chatbot when patient finish speaking. Set a stoping flag
            return
        logger.info("At this time user is speaking, need wait (do nothing) till he finish")
    # When ASR receives a full sentence, and decided to send it to chat bot
    elif user_speaking_state == 3:
        sentence_heard = time_window.get_cached_sentences()
        logger.info(f"User stoped speaking as word stream input timeout. Start to trigger chatbot. \nSentence heard: {sentence_heard}")

        default_action = multithread_action_wrapper()
        default_action.start()


    # If patient is not speaking now, we think of replies.
    elif user_speaking_state == 0:
        # Handle the emergency stop when patient stop speaking
        if emergency_stop_flag:
            logger.info("Emergency stop due to agitation")
            # TODO: gracefully stop the robot. here I only log the message
            # currently I write a return here, though it won't actually stop the loop
            return
        
        # Patient don't answer with in time_window in if-else branch
        # Check engagement level
        elif engagement_state == "Distracted":
            logger.info("No feedback from patients and patient is distracted, asking robot to repeat")
            # TODO: ask chatbot to repeat the question once
            # 1. send an artificial message to chatbot: "Can you repeat?"
            # 2. await for chatbot's response
            #time.sleep(2)
            return
        elif engagement_state == "Agitated":
            logger.info("Patient didn't answer and is agitated, ask robot to gracefully stop at once")
            # TODO: ask chatbot to repeat the question once
            # 1. send an artificial message to chatbot: "I don't want to talk anymore"
            # 2. emergency stop
            emergency_stop_flag = True
            return
    else:
        logger.error(f"user speaking state not in 0,1,2,3, is {user_speaking_state}")
        sys.exit(-1)

        
        
        


        



if __name__ == "__main__":

    #Yifan edit:
    main_loop_node = rospy.init_node("main_loop")

    # load configs
    args = load_config("./config/config_full10.json")

    # setup logger
    logger = setup_logger()

    # listener for start button
    # currently not integrated to the mainloop
    start_command = start_command_listener(args, logger)

    # listener for ASR
    asr_listener = ASR_Full_Sentence(args, logger)

    # Emotion listener
    emotion_listener = Emotion_Recognition_Handeler(args)

    # time_window
    time_window = time_window_manager(args, time_window=0.5)

    # engagement estimator
    em = engagement_estimator(emotion_listener)

    # Connector to robot
    try:
        robot_connector = action_trigger()
    except rospy.exceptions.ROSException as e:
        logger.error("Fail to connect to robot's custom action service \n" + str(e))
        sys.exit(0) # comment this line if want to start mainloop without custom action APIs


    #Yifan note: no need to actively call the listener
    # start_command.listen()

    #Speaking state var
    user_speaking_state = False

    # Whether Grace is speaking
    robot_speaking = False
    # Whether to have emergency stop
    emergency_stop_flag = False
    # whether robot is interrupted
    hardware_interrupt = False

    #Yifan note: put an infinite loop here just for testing
    rate = rospy.Rate(10)#30hz

    # make sure that ctrl-c can work
    while not rospy.is_shutdown():
        main_loop()
        rate.sleep()#Will make sure this loop runs at 30Hz



    # ###Start Button callback from GUI###
    # start = False
    # while args.start_button == True: # if a start button is implemented
    #     pass
    #     if start == True:
    #         break

    # ### Start ASR module to listen for full sentence
    # asr_handler = ASR_Full_Sentence()
    # asr_handler.listen()

    # main_loop(asr_handler)
