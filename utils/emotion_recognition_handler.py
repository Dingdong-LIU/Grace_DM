import rospy
import grace_attn_msgs
from collections import deque, Counter

class Emotion_Recognition_Handeler:
    def __init__(self, args) -> None:
        #rospy.init_node("emotion_recognition_node")
        self.emotion_sub = rospy.Subscriber(
            args.ros_topic["emotion_topic"],
            grace_attn_msgs.msg.EmotionAttentionResult,
            callback=self.callback,
            queue_size=100
        )
        self.attention = deque([""]*10, maxlen=10)
        self.emotion = deque([""]*10, maxlen=10)
        self.visualization_frame = None # This one is of no use, so not updated


    def callback(self, msg):
        # Do with emotion messages here
        self.attention.appendleft(msg.attention)
        self.emotion.appendleft(msg.emotion)
        # self.visualization_frame = msg.visualization_frame
        # need to send to mainloop to trigger something.
    
    def get_signal_state(self, signal_name):
        """Get the singal value from emotion recognition subscriber. This method perform a voting of the most common signals within 10 samples

        Args:
            signal_name (str): either "attention" or "emotion"

        Returns:
            str: The most common signal in 10 most recent calls. "attention" could be "True" or "False"; "emotion" could be {'Anger', 'Contempt', 'Agitation' ('Disgust', 'Fear'), 'Happiness', 'Neutral', 'Sadness', 'Surprise', "Abscence"(no face), "EXCEPTION!!"(didn't track patient)}
        """
        assert signal_name in ["attention", "emotion"], f"{signal_name} is not supported"
        queue_object = None
        if queue_object == "attention":
            queue_object = self.attention
        elif queue_object == "emotion":
            queue_object = self.emotion
        occurence_count = Counter(queue_object)
        attn = occurence_count.most_common(1)
        if attn != "":
            return attn[0][0]
        else:
            attn = occurence_count.most_common(2)
            return attn[1][0]
    

