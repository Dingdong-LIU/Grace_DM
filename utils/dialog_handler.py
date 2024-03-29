import requests 
import random
from logging import getLogger

NGROK_LINK = 'https://c0c9-143-89-145-170.ngrok-free.app'
class DIALOG_HANDLR:
    def __init__(self) -> None:
        self.session_id = random.randint(10000000, 500000000)
        self.logger = getLogger()

    def test(self):
        print(self.session_id)

    def communicate(self, asr_text):
        self.logger.debug("Start to communicate with chatbot: %s" % asr_text)
        response = requests.post(
            f"{NGROK_LINK}/dialogflow_result",
            json={
                "text": asr_text,
                "session_id": self.session_id
            }
        )
        self.logger.debug("Received replies from chatbot: %s" % str(response.json()))
        return response.json()