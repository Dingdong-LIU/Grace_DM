import rospy
import hr_msgs.msg 
import hr_msgs.srv
import std_msgs.msg
import grace_attn_msgs.msg
from config.load_config import load_config
from utils.asr_handler import ASR_Full_Sentence
from utils.log_manager import setup_logger
from utils.start_command_listener import start_command_listener
from utils.mainloop_manager import time_window_manager



def main_loop():
    """ This is one loop happens in the main frame.
    Basically there are 2 stages, listening stage & speaking stage.

    Args:
        asr (_type_): _description_
    """

    ## print(asr_listener.asr_full_sentence) # For debug Only

    # If patient is speaking, we only run emotion and vision analysis. No action will happen in this time.

    # If patient is not speaking now, we think of replies.




if __name__ == "__main__":
    # load configs
    args = load_config("./config/config_full10.json")

    # setup logger
    logger = setup_logger()

    # listener for start button
    # currently not integrated to the mainloop
    start_command = start_command_listener(args)

    # listener for ASR
    asr_listener = ASR_Full_Sentence(args)

    # time_window
    time_window = time_window_manager(args, time_window=4)

    #Yifan note: no need to actively call the listener
    # start_command.listen()

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
