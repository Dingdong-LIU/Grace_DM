import rospy
import dynamic_reconfigure.client
import hr_msgs


class ASR_Full_Sentence:
    def __init__(self) -> None:
        self.asr_language_config = self.ros_dynamic_configuration(lang="EN")

        # Subscriber listening to ROS topic
        self.full_sentence_listener = None
        # Variables to be filled
        self.asr_full_sentence = None
        self.sentence_format = {
            "lang" : "", "confidence": 0,
            "source" : "", "audio_path": "",
        }

    def ros_dynamic_configuration(self, lang):
        """Dynamic configuration of ASR language settings, 
        choose between Cantonese yue-Hant-HK and English en-GB"""
        rospy.init_node('myconfig_py', anonymous=True)
        client = dynamic_reconfigure.client.Client('/hr/perception/speech_recognizer')    
        if lang=="EN":
            params = { 'enable': True, 'language':'en-GB'} #'en-GB'
        else:
            params = { 'enable': True, 'language':'yue-Hant-HK'} #'yue-Hant-HK'
        config = client.update_configuration(params)
        return config

    def ASR_sentence_fallback(self, msg):
        self.asr_full_sentence = msg.utterance
        self.sentence_format["lang"] = msg.lang
        self.sentence_format["confidence"] = msg.confidence
        self.sentence_format["source"] = msg.source
        self.sentence_format["audio_path"] = msg.audio_path
        
    
    def listen(self, args):
        self.full_sentence_listener = rospy.Subscriber(
            args.ros_topic['ASR_full_sentence'], 
            hr_msgs.msg.ChatMessage, 
            self.ASR_sentence_fallback
        )