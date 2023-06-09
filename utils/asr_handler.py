import rospy
import dynamic_reconfigure.client
import hr_msgs
import time
from logging import Logger, getLogger

class ASR_Word_Stream:
    """This class listens to ASR word stream.
    The class will store listened word and timestamp it sees an word input.
    """
    def __init__(self, args) -> None:
        """Create a subscriber listen to ASR word stream - the instant word from ASR module.

        Args:
            args (Namespace): should contain a "tos_topic" namespace, which is a dict that has key "ASR_word" refer to path of ASR_word_stream
        """
        #rospy.init_node("ASR_Word_Stream")
        self.word_listener = rospy.Subscriber(
            args.ros_topic["ASR_word"],
            hr_msgs.msg.ChatMessage,
            self.callback,
            queue_size=100
        )
        self.word = ""
        self.timestamp = 0
        self.new_word = False
        self.logger = getLogger()

    def callback(self, msg):
        self.word = msg.utterance
        self.timestamp = time.time()
        self.new_word = True
        self.logger.info(f"ASR_WORD_STREAM: {self.word}")
    
    def get_time_stamp(self):
        output = (self.new_word, self.timestamp)
        self.new_word = False
        return output


class ASR_Sentence_Stream:
    def __init__(self) -> None:
        pass



class ASR_Full_Sentence:
    def __init__(self, args) -> None:

        # This is the configuration of ASR. I comment this out as Yifan told me he will handle it on his side. If there's misunderstanding, please uncomment this line.
        # self.asr_language_config = self.ros_dynamic_configuration(lang="HK")

        # Subscriber listening to ROS topic
        #rospy.init_node("ASR_Full_Sentence_Node")
        self.full_sentence_listener = rospy.Subscriber(
            args.ros_topic['ASR_full_sentence'], 
            hr_msgs.msg.ChatMessage, 
            self.ASR_sentence_callback,
            queue_size=100
        )
        # Variables to be filled
        self.asr_full_sentence = None
        self.sentence_format = {
            "lang" : "", "confidence": 0,
            "source" : "", "audio_path": "",
        }
        self.logger = getLogger()

        self.new_sentence = False
        self.timestamp = 0

    def ros_dynamic_configuration(self, lang="HK"):
        """Dynamic configuration of ASR language settings, 
        choose between Cantonese yue-Hant-HK and English en-GB"""
        #rospy.init_node('myconfig_py', anonymous=True)
        client = dynamic_reconfigure.client.Client('/hr/perception/speech_recognizer')    
        if lang=="EN":
            params = { 'enable': True, 'language':'en-GB'} #'en-GB'
        else:
            params = { 'enable': True, 'language':'yue-Hant-HK'} #'yue-Hant-HK'
        config = client.update_configuration(params)
        return config

    def ASR_sentence_callback(self, msg):
        # Update the asr message storage
        self.asr_full_sentence = msg.utterance
        self.sentence_format["lang"] = msg.lang
        self.sentence_format["confidence"] = msg.confidence
        self.sentence_format["source"] = msg.source
        self.sentence_format["audio_path"] = msg.audio_path


        self.logger.info(f"ASR_SENTENCE_STREAM: Obtained sentence ({self.asr_full_sentence}) from ASR")

        self.new_sentence = True
        self.timestamp = time.time()


    def get_full_sentence(self):
        if self.new_sentence:
            self.new_sentence = False
            wait = False
            return (wait, self.asr_full_sentence)
        else:
            wait = True
            return (wait, self.asr_full_sentence)
    
    def get_time_stamp(self):
        output = (self.new_sentence, self.timestamp)
        self.new_sentence = False
        return output


