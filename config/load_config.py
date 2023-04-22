from pathlib import Path
import json
import argparse

def load_config():
    json_file_path = "config_full10.json"
    json_text = Path(json_file_path).read_text()
    config = json.loads(json_text)
    parser = argparse.ArgumentParser(
        description="Settings for ROS Bridge, talker side")
    parser.add_argument("--topic", required=False, type=str,  default="/grace_performance",
                        help="Name of the topic to establish connection between the talker and the listener. (e.g. '\grace_chat')")
    parser.add_argument("--settings", required=False, type=str,
                        default="settings_yue_performance",  help="Conversation settings")
    parser.add_argument("--folder", required=False, type=str, default="second_stage",
                        help="Name of the parent folder containing all the performances")
    parser.add_argument("--start_button", required=False, type=bool,
                        default=False, help="Informs whether the start button is needed")
    args = parser.parse_args()
    args = argparse.Namespace(**config)
    return args
