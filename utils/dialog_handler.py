import requests 
import random

NGROK_LINK = 'https://b6cf-143-89-145-170.ngrok-free.app'
class DIALOG_HANDLR:
    def __init__(self) -> None:
        self.session_id = random.randint(10000000, 500000000)

    def test(self):
        print(self.session_id)  

    def communicate(self, asr_text):
        response = requests.post(
            f"{NGROK_LINK}/dialogflow_result",
            json={
                "text": asr_text,
                "session_id": self.session_id
            }
        )
        return response.json()