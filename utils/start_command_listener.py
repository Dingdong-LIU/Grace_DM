import rospy
import std_msgs

class start_command_listener:
    """ Listen for the start command
    """
    def __init__(self, args) -> None:
        """A subscriber for "start_topic", which should be triggered by GUI. 

        Args:
            args (Namespace): Should contain a ros_topic field which is a dict containing key "start_topic". "start_topic" is the key to path of start topic in ROS.
        """
        self.start = False
        rospy.init_node("start_command_node")
        self.start_sub = rospy.Subscriber(
            args.ros_topic['start_topic'], std_msgs.msg.Bool, 
            callback=self.start_of_conversation_callback, 
            queue_size=100
        )

        #Yifan note: setup a rosnode
        # rospy.init_node("start_command_node")
        # #Yifan note: create a subscriber object where we specify the topic, the type of message, the callback and finally the queue size
        # self.asr_words_sub = rospy.Subscriber("/hr/perception/hear/words", hr_msgs.msg.ChatMessage, self.listen, queue_size=100)

    def start_of_conversation_callback(self, msg) -> None:
        # Broadcast a start signal to everyone
        self.start = msg.data
    
    def get_state(self) -> bool:
        """Return state of the start command

        Returns:
            bool: True for start; False for not start
        """
        return self.start

    #Yifan note: this is the callback that would be invoked whenever a message arrives
    # def listen(self, msg):
    #     print(msg.utterance)
    #     self.start_sub = rospy.Subscriber(
    #         args.ros_topic['start_topic'], std_msgs.msg.Bool, 
    #         callback=self.start_of_conversation_callback, 
    #         queue_size=args.topic_queue_size
    #     )