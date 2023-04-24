import rospy
import hr_msgs.msg 
import hr_msgs.srv
import std_msgs.msg
import grace_attn_msgs.msg
from config.load_config import load_config
from utils.asr_handler import ASR_Full_Sentence
from utils.log_manager import setup_logger
from utils.start_command_listener import start_command_listener
from utils.mainloop_manager import time_window_manager, engagement_estimator
import time
from utils.robot_trigger import action_trigger



def main_loop():
    """ This is one loop happens in the main frame.
    Basically there are 2 stages, listening stage & speaking stage.

    Args:
        asr (_type_): _description_
    """

    ## print(asr_listener.asr_full_sentence) # For debug Only

    ## Check if Grace is speaking, then don't do anything except for tracking engagement level
    if robot_speaking:
        if engagement_state == "Agitated":
            # only handle "Agitated" when patient is speaking
            logger.debug("Currently Agitated")
            # TODO: Do something
            # Stop the orientation
            # 1. Pass an emergency stop to Grace. 
            # 2. Stop the chatbot when patient finish speaking. Set a stoping flag

            pass
        return

    speaking_state = time_window.check_asr_input()
    # If patient is speaking, we only run emotion and vision analysis. We will wait for chatbot to generate a reply.
    if speaking_state:
        engagement_state = em.update_engagement_level()
        if engagement_state == "Agitated":
            # only handle "Agitated" when patient is speaking
            logger.debug("Currently Agitated")
            # TODO: Do something
            # Stop the orientation
            # 1. Pass an emergency stop to Grace. 
            # 2. Stop the chatbot when patient finish speaking. Set a stoping flag
            return
        
        # TODO: An await function to wait for chatbot reply.
        # await for asr_listener
        time.sleep(2)
        # TEST:
        # robot_connector.test_send_request()

    # If patient is not speaking now, we think of replies.
    else:
        # Handle the emergency stop when patient stop speaking
        if emergency_stop_flag:
            logger.debug("Emergency stop due to agitation")
            # TODO: gracefully stop the robot. here I only log the message
            # currently I write a return here, though it won't actually stop the loop
            return
        
        # Patient don't answer with in time_window in if-else branch
        # Check engagement level
        engagement_state = em.update_engagement_level()
        if engagement_state == "Distracted":
            logger.info("No feedback from patients and patient is distracted, asking robot to repeat")
            # TODO: ask chatbot to repeat the question once
            # 1. send an artificial message to chatbot: "Can you repeat?"
            # 2. await for chatbot's response
            time.sleep(2)
            return
        if engagement_state == "Agitated":
            logger.info("Patient didn't answer and is agitated, ask robot to gracefully stop at once")
            # TODO: ask chatbot to repeat the question once
            # 1. send an artificial message to chatbot: "I don't want to talk anymore"
            # 2. emergency stop
            emergency_stop_flag = True
            return

        
        
        


        




if __name__ == "__main__":
    # load configs
    args = load_config("./config/config_full10.json")

    # setup logger
    logger = setup_logger()

    # listener for start button
    # currently not integrated to the mainloop
    start_command = start_command_listener(args, logger)

    # listener for ASR
    asr_listener = ASR_Full_Sentence(args, logger)

    # time_window
    time_window = time_window_manager(args, time_window=2)

    # engagement estimator
    em = engagement_estimator()

    # Connector to robot
    robot_connector = action_trigger()

    #Yifan note: no need to actively call the listener
    # start_command.listen()

    # Whether Grace is speaking
    robot_speaking = False
    # Whether to have emergency stop
    emergency_stop_flag = False

    #Yifan note: put an infinite loop here just for testing
    rate = rospy.Rate(30)#30hz
    while True:
        main_loop(asr_listener)
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
