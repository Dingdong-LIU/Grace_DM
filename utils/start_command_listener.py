import rospy
import std_msgs

class start_command_listener:
    """ Listen for the start command
    """
    def __init__(self, args) -> None:
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

    def start_of_conversation_callback(self, msg):
        # Broadcast a start signal to everyone
        self.start = msg.data

    #Yifan note: this is the callback that would be invoked whenever a message arrives
    # def listen(self, msg):
    #     print(msg.utterance)
    #     self.start_sub = rospy.Subscriber(
    #         args.ros_topic['start_topic'], std_msgs.msg.Bool, 
    #         callback=self.start_of_conversation_callback, 
    #         queue_size=args.topic_queue_size
    #     )