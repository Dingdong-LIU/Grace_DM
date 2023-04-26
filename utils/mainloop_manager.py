import rospy
from utils.asr_handler import ASR_Word_Stream, ASR_Full_Sentence
import time
from utils.emotion_recognition_handler import Emotion_Recognition_Handeler
import logging

class time_window_manager:
    """This class tells the mode of Grace: ASR or NO_ASR. The class maintains an ASR_Word_Stream to monitor if there's input from user.

    bool state -> False="ASR state"; True="NO_ASR state"

    """
    def __init__(self, args, time_window=0.5) -> None:
        """Init function. Create a ASR_WORD_STREAM subscriber.

        Args:
            args (Namespace): args for ASR_WORD_STREAM. Should contain a "tos_topic" namespace, which is a dict that has key "ASR_word" refer to path of ASR_word_stream.
            time_window (int, optional): Timewindow for changing state. Defaults to 0.5 (seconds).
        """
        self.state = 0
        self.logger = logging.getLogger()
        self.last_update_time = 0
        self.asr_word_stream = ASR_Word_Stream(args)
        self.asr_full_sentence = ASR_Full_Sentence(args, self.logger)
        self.timeout = time_window
        self.heard_sentences = []


    def check_asr_input(self) -> int:
        """Check what is the ASR state with time_window as timeout.

        Returns:
            int: 0: no speaking; 1 heard words, the user is speaking; 2 heard sentence, the user is speaking 3 submit to chatbot state (should be very short).
        """
        current_time = time.time()
        new_word, last_word_time = self.asr_word_stream.get_time_stamp()
        new_sentence, last_sentence_time = self.asr_full_sentence.get_time_stamp()
        if self.state == 0: # no speaking state
            if new_sentence:
                # enter the "heard sentence state", cache sentences
                self.state = 2
                self.heard_sentences.append(self.asr_full_sentence.get_full_sentence()[1])
            elif not new_sentence and new_word:
                self.state = 1
            else:
                self.state = 0
        elif self.state == 1:
            if new_sentence: 
                # enter the "heard sentence state", start to cache sentences
                self.state = 2
                self.heard_sentences.append(self.asr_full_sentence.get_full_sentence()[1])
            elif not new_sentence and new_word:
                self.state = 1
            elif current_time - last_word_time > self.timeout: # check for timeout
                self.state = 0
        elif self.state == 2:
            if new_sentence:
                self.state = 2
                self.heard_sentences.append(self.asr_full_sentence.get_full_sentence()[1])
            elif not new_sentence and new_word:
                self.state = 2
            elif current_time - last_word_time > self.timeout:
                #timeout for "heard sentence state"
                self.state = 3 # need to submit to chatbot at once. Call get_cache_sentences to clear the cache
        else: # self.state==3
            self.state = 0

        return self.state

    def get_cached_sentences(self):
        sentence = " ".join(self.heard_sentences)
        self.heard_sentences = []
        return sentence


class engagement_estimator:
    """This class reades ASR, Attention, Emotion inputs and decides the strategy to use. This class runs at slower frequency (around 30hz by default settings)
    """
    def __init__(self, emotion_module:Emotion_Recognition_Handeler) -> None:
        self.attention = None
        self.asr = None
        self.emotion = None
        self.state = None
        self.emotion_module = emotion_module

    def update_engagement_level(self) -> str:
        """Check the sensor cache and determine the patient's engagement level

        Args:
            asr_module (Emotion_Recognition_Handeler): The emotional subscriber listening to emotion message.

        Returns:
            str: Either "Engaged", "Distracted" or "Agitated" or "NOT_STABLE". NOT_STABLE means input from emotion or attention is empty
        """
        engagement_level = "NOT_STABLE"
        emotion = self.emotion_module.get_signal_state("emotion")
        attention = self.emotion_module.get_signal_state("attention")
        #Ugly fix for agitation
        # if emotion in ["Anger", "Agitation"]:
        if emotion in ["NONEXISTENT"]:
            engagement_level = "Agitated"
        elif attention == "False":
            engagement_level = "Distracted"
        elif emotion in ["Abscence", "EXCEPTION!!"]:
            engagement_level = "Distracted"
        elif emotion == "" or attention == "":
            engagement_level = "NOT_STABLE"
        else:
            engagement_level = "Engaged"
        
        return engagement_level
    
        # engagement_level = 
