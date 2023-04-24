import rospy
from asr_handler import ASR_Word_Stream
import time
from emotion_recognition_handler import Emotion_Recognition_Handeler

class time_window_manager:
    """This class tells the mode of Grace: ASR or NO_ASR. The class maintains an ASR_Word_Stream to monitor if there's input from user.

    bool state -> False="ASR state"; True="NO_ASR state"

    """
    def __init__(self, args, time_window=1) -> None:
        """Init function. Create a ASR_WORD_STREAM subscriber.

        Args:
            args (Namespace): args for ASR_WORD_STREAM. Should contain a "tos_topic" namespace, which is a dict that has key "ASR_word" refer to path of ASR_word_stream.
            time_window (int, optional): Timewindow for changing state. Defaults to 1 (seconds).
        """
        self.state = False
        self.last_update_time = 0
        self.asr_word_stream = ASR_Word_Stream(args)
        self.time_window = time_window

    def asr_input(self) -> None:
        """Change state to TRUE (ASR state)
        """
        self.state = True
    
    def no_asr(self) -> None:
        """Change state to False (NO_ASR state)
        """
        self.state = False

    def check_asr_input(self) -> bool:
        """Check if there is ASR input in the time_window (4s)

        Returns:
            bool: True->ASR_Input; False->NO_ASR
        """
        # no words in time_window (default 4s) seconds
        if time.time() - self.last_update_time > self.time_window:
            self.no_asr()
        else:
            self.asr_input()
        self.last_update_time = self.asr_word_stream.timestamp
        return self.state


class engagement_estimator:
    """This class reades ASR, Attention, Emotion inputs and decides the strategy to use. This class runs at slower frequency (around 30hz by default settings)
    """
    def __init__(self) -> None:
        self.attention = None
        self.asr = None
        self.emotion = None
        self.state = None

    def update_engagement_level(self, asr_module:Emotion_Recognition_Handeler) -> str:
        engagement_level = "Engaged"
        if asr_module.get_signal_state("attention") == "False":
            engagement_level = "Distracted"
        if asr_module.get_signal_state("emotion") in ["Anger", "Agitation"]:
            engagement_level = "Agitated"
        return engagement_level
    

