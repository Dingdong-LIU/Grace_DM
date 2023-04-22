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
    #Yifan note: buggy, comment out
    #logger.info(f"ASR_FULL_SENTENCE:{asr.asr_full_sentence}")

class start_command_listener:
    """ Listen for the start command
    """
    def __init__(self) -> None:
        self.start = False
        self.start_sub = None

        #Yifan note: setup a rosnode
        rospy.init_node("some_node_name")
        #Yifan note: create a subscriber object where we specify the topic, the type of message, the callback and finally the queue size
        self.asr_words_sub = rospy.Subscriber("/hr/perception/hear/words", hr_msgs.msg.ChatMessage, self.listen, queue_size=100)





    def start_of_conversation_callback(self, msg):
        # Broadcast a start signal to everyone
        self.start = msg.data

    #Yifan note: this is the callback that would be invoked whenever a message arrives
    def listen(self, msg):
        print(msg.utterance)
        # self.start_sub = rospy.Subscriber(
        #     args.ros_topic['start_topic'], std_msgs.msg.Bool, 
        #     callback=self.start_of_conversation_callback, 
        #     queue_size=args.topic_queue_size
        # )


if __name__ == "__main__":
    # load configs
    args = load_config()

    #Yifan note: buggy, comment out
    # # setup logger
    # logger = setup_logger()

    # listen for start button
    start_command = start_command_listener()

    #Yifan note: no need to actively call the listener
    # start_command.listen()

    #Yifan note: put an infinite loop here just for testing
    rate = rospy.Rate(30)#30hz
    while True:
        rate.sleep()#Will make sure this loop runs at 30Hz


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
