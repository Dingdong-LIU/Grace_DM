import rospy
import grace_attn_msgs

class Emotion_Recognition_Handeler:
    def __init__(self, args) -> None:
        rospy.init_node("emotion_recognition_node")
        self.emotion_sub = rospy.Subscriber(
            args.ros_topic["emotion_topic"],
            grace_attn_msgs.msg.EmotionAttentionResult,
            callback=self.callback,
            queue_size=100
        )
        self.attention = None
        self.emotion = None
        self.visualization_frame = None # This one is of no use, so not updated


    def callback(self, msg):
        # Do with emotion messages here
        self.attention = msg.attention
        self.emotion = msg.emotion
        self.visualization_frame = msg.visualization_frame
        # need to send to mainloop to trigger something.
