import rospy
import hr_msgs.msg 
import hr_msgs.srv
import std_msgs.msg
import grace_attn_msgs.msg
from config.load_config import load_config
from utils.asr_handler import ASR_Full_Sentence
from utils.log_manager import setup_logger

# need to listen for start/stop



def main_loop(asr):
    print(asr.asr_full_sentence)
    logger.info(f"ASR_FULL_SENTENCE:{asr.asr_full_sentence}")

class start_command_listener:
    """ Listen for the start command
    """
    def __init__(self) -> None:
        self.start = False
        self.start_sub = None

    def start_of_conversation_callback(self, msg):
        # Broadcast a start signal to everyone
        self.start = msg.data

    def listen(self, args):
        self.start_sub = rospy.Subscriber(
            args.ros_topic['start_topic'], std_msgs.msg.Bool, 
            callback=self.start_of_conversation_callback, 
            queue_size=args.topic_queue_size
        )


if __name__ == "__main__":
    # load configs
    args = load_config()

    # setup logger
    logger = setup_logger()

    # listen for start button
    start_command = start_command_listener()
    start_command.listen()

    ###Start Button callback from GUI###
    start = False
    while args.start_button == True: # if a start button is implemented
        pass
        if start == True:
            break

    ### Start ASR module to listen for full sentence
    asr_handler = ASR_Full_Sentence()
    asr_handler.listen()

    main_loop(asr_handler)
