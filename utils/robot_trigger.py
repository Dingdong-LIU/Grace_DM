import rospy
import yaml
from argparse import Namespace

import grace_attn_msgs.msg
import grace_attn_msgs.srv
import hr_msgs.msg
import hr_msgs.cfg
import hr_msgs.srv
import std_msgs


class action_trigger:
    def __init__(self) -> None:
        self.grace_api_configs = self.loadConfigs()
        #Ros routine
        #self.node = rospy.init_node("exec_test")
        rospy.wait_for_service(self.grace_api_configs['Ros']['grace_behavior_service'], timeout=3)
        self.grace_behavior_client = rospy.ServiceProxy(self.grace_api_configs['Ros']['grace_behavior_service'], grace_attn_msgs.srv.GraceBehavior)

        self.req = grace_attn_msgs.srv.GraceBehaviorRequest()

    def open_database(self):
        return 
    def test_send_request(self):
        print("start to wait for service to finish")
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
        print("start to wait for service to finish")
        success_state = self.grace_behavior_client(req)
        print("Service call response is:\n %s" % success_state)
        # rospy.wait_for_service(self.grace_api_configs['Ros']['grace_behavior_service'], timeout=6)


    #Load configs
    def loadConfigs(self):
        #Load configs
        with open("/home/grace_team/HKUST_GRACE/Grace_Project/Grace_DM/config/robot_api.yaml", "r") as config_file:
            grace_api_configs = yaml.load(config_file, Loader=yaml.FullLoader)
            print("Read successful")
        return grace_api_configs
    
    def loopup_sentence_configs(self, sentence:str) -> grace_attn_msgs.srv.GraceBehaviorRequest:

        return self.req



class stop_trigger:
    def __init__(self, args:Namespace) -> None:
        self.stop_topic = args.ros_topic["stop_topic"]
        self.topic_queue_size = args.topic_queue_size

        self.request = hr_msgs.srv.RunByNameRequest()

        self.stop_sub = rospy.Subscriber(self.stop_topic, std_msgs.msg.Bool, self.stop_msg_callback, queue_size=self.topic_queue_size)
        self.stop_pub = rospy.Publisher(self.stop_topic , std_msgs.msg.Bool, queue_size=self.topic_queue_size)
        self.start=False
    
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





