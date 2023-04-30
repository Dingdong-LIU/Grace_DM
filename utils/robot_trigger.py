import rospy
import yaml
from argparse import Namespace

import grace_attn_msgs.msg
import grace_attn_msgs.srv
import hr_msgs.msg
import hr_msgs.cfg
import hr_msgs.srv
import std_msgs
import logging
from utils.data_reader import database_reader

class action_trigger:
    def __init__(self, lang='yue-Hant-HK') -> None:
        self.logger = logging.getLogger()
        self.grace_api_configs = self.loadConfigs()
        #Ros routine
        #self.node = rospy.init_node("exec_test")
        rospy.wait_for_service(self.grace_api_configs['Ros']['grace_behavior_service'], timeout=3)
        self.grace_behavior_client = rospy.ServiceProxy(self.grace_api_configs['Ros']['grace_behavior_service'], grace_attn_msgs.srv.GraceBehavior)
        self.lang = lang

        self.req = grace_attn_msgs.srv.GraceBehaviorRequest()
        # datareader for performance config
        self.database_reader = database_reader(filename="/home/grace_team/HKUST_GRACE/Grace_Project/Grace_DM/data/intent_emotion_mapping.xlsx")
    
    def parse_reply_from_chatbot(self, res:dict):
        intent = res["responses"]['intent']
        utterance = res["responses"]['text']
        params = self.database_reader.lookup_table(intent_name=intent)
        return (utterance, params)
    

    def compose_req(self, command:str, utterance:str, params:dict) -> grace_attn_msgs.srv.GraceBehaviorRequest:
        """Compose a string using params and command

        Args:
            command (str): Command for Grace Robot, can only be 'exec' or 'stop'
            utterance (str): A string for Grace to speak
            params (dict): a set of params

        Returns:
            grace_attn_msgs.srv.GraceBehaviorRequest: a request
        """
        self.req = grace_attn_msgs.srv.GraceBehaviorRequest(**params)
        if utterance == "":
            utterance = "please repeat"
        self.req.utterance = utterance
        self.req.command = command
        self.req.lang = self.lang
        return self.req
    
    def send_request(self, req:grace_attn_msgs.srv.GraceBehaviorRequest) -> str:
        """execute the request and send reply

        Args:
            req (grace_attn_msgs.srv.GraceBehaviorRequest): can be (1) "completed", meaning that Grace completed the execution without noticing any interruption (2) "interrupted", meaning that Grace was interruped by her interlocutor and has to stop the speech execution as a reaction mid-way. or (3) "stopped", meaning the stop behavior command has been executed.
        Returns:
            _type_: _description_
        """
        success_state = self.grace_behavior_client(req).result
        return success_state

    def test_send_request(self):
        print("start to wait for service to finish")
        # some_command = {'expressions': 'happy', 'exp_start': 20, 'exp_end': 80, 'exp_mag': 0.85}
        # grace_attn_msgs.srv.GraceBehaviorRequest(**some_command)
        req = grace_attn_msgs.srv.GraceBehaviorRequest()
        req.command = 'exec'
        req.utterance = self.grace_api_configs['Debug']['Sample']['txt']
        req.lang = self.grace_api_configs['Debug']['Sample']['lang']

        req.expressions = self.grace_api_configs['Debug']['Sample']['expressions']
        req.exp_start = self.grace_api_configs['Debug']['Sample']['exp_start']
        req.exp_end = self.grace_api_configs['Debug']['Sample']['exp_end']
        req.exp_mag = self.grace_api_configs['Debug']['Sample']['exp_mag']

        req.gestures = self.grace_api_configs['Debug']['Sample']['gestures']
        req.ges_start = self.grace_api_configs['Debug']['Sample']['ges_start']
        req.ges_end = self.grace_api_configs['Debug']['Sample']['ges_end']
        req.ges_mag = self.grace_api_configs['Debug']['Sample']['ges_mag']

        #Call the service
        self.logger.info("start to wait for service to finish")
        success_state = self.grace_behavior_client(req).result
        self.logger.info("Service call response is:\n %s" % success_state)
        # rospy.wait_for_service(self.grace_api_configs['Ros']['grace_behavior_service'], timeout=6)

        return success_state


    #Load configs
    def loadConfigs(self):
        #Load configs
        with open("/home/grace_team/HKUST_GRACE/Grace_Project/Grace_DM/config/robot_api.yaml", "r") as config_file:
            grace_api_configs = yaml.load(config_file, Loader=yaml.FullLoader)
            self.logger.info("Read successful")
        return grace_api_configs
    
    def stop_conversation(self, error_message:str):
        req = grace_attn_msgs.srv.GraceBehaviorRequest()
        req.command = 'stop'
        self.grace_behavior_client(req)
        self.logger.error(f"Emergency Stop because {error_message}")



class stop_trigger:
    def __init__(self, args:Namespace) -> None:
        self.stop_topic = args.ros_topic["stop_topic"]
        self.topic_queue_size = args.topic_queue_size

        self.request = hr_msgs.srv.RunByNameRequest()

        self.stop_sub = rospy.Subscriber(self.stop_topic, std_msgs.msg.Bool, self.stop_msg_callback, queue_size=self.topic_queue_size)
        self.stop_pub = rospy.Publisher(self.stop_topic , std_msgs.msg.Bool, queue_size=self.topic_queue_size)
        self.stop_message=False
    
    def stop_msg_callback(self, msg):
        # Need a message to call the emergency stop function
        self.stop_message = msg.data
        
        # if(self.stop_message == True):
        #     self.force_stop = True #Set the flag
        #     #Display graceful stop performance
        #     disengage_stop = self.folder + "/disengage_stop/" + random.choice(self.conv_settings["disengage_stop"])
        #     self.request.id =  disengage_stop
        #     response = self.run_by_name_handle(self.request)
        #     #Kill the current process
        #     os.system('kill ' + str(os.getpid()))

    def pub_stop(self):
        self.stop_pub.publish(std_msgs.msg.Bool(True))


