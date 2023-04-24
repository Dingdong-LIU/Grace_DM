import rospy
import yaml

import grace_attn_msgs.msg
import grace_attn_msgs.srv
import hr_msgs.msg
import hr_msgs.cfg
import hr_msgs.srv


class action_trigger:
    def __init__(self) -> None:
        self.grace_api_configs = self.loadConfigs()
        #Ros routine
        #self.node = rospy.init_node("exec_test")
        self.grace_behavior_client = rospy.ServiceProxy(self.grace_api_configs['Ros']['grace_behavior_service'], grace_attn_msgs.srv.GraceBehavior)

    def test_send_request(self):
        req = grace_attn_msgs.srv.GraceBehaviorRequest()
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
        success_state = self.grace_behavior_client(req)
        print("Service call response is:\n %s" % success_state)



    #Load configs
    def loadConfigs(self):
        #Load configs
        with open("./config/robot_api.yaml", "r") as config_file:
            grace_api_configs = yaml.load(config_file, Loader=yaml.FullLoader)
            print("Read successful")
        return grace_api_configs






