import json
from langchain_community.chat_message_histories import ChatMessageHistory
from langchain_core.chat_history import BaseChatMessageHistory
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain_core.runnables.history import RunnableWithMessageHistory
from langchain_ollama import ChatOllama
from langchain_core.messages import HumanMessage, SystemMessage

import base64
from io import BytesIO
# from IPython.display import HTML, display
from PIL import Image

class QA_Agent:
    def __init__(self):
        self.llm = ChatOllama(model="llava")
        print("Model Loaded")

    def convert_to_base64(self, file_path):
        """
        Convert PIL images to Base64 encoded strings

        :param pil_image: PIL image
        :return: Re-sized Base64 string
        """
        pil_image = Image.open(file_path)
        buffered = BytesIO()
        pil_image.save(buffered, format="PNG")  # You can change the format if needed
        img_str = base64.b64encode(buffered.getvalue()).decode("utf-8")
        return img_str

    def get_system_prompt(self):
        system_prompt = (
            """
            You are a Kuka robotic arm. User will pass you images and ask you to perform actions
            with respect to gripper position.
            Provide user with steps that you will take to perform the said action. To perform the
            action use the functions [move, rotate]
            Example:
            User: Move left 3
            Steps:
                1. Move(0, -3, 0)
            
            User: Turn left then go 7 units down
            Steps:
                1. rotate(-1.5707)
                2. move(0, 0, -7)
            """
        )
        return system_prompt
    
    def agent_chat(self, usr_prompt, img_path=None):
        image = self.convert_to_base64(img_path)
        input = [{"type": "image_url", "image_url": f"data:image/png;base64,{image}"}, {"type": "text", "text": usr_prompt}]
        prompt = [
            SystemMessage(self.get_system_prompt()),
            HumanMessage(input)
        ]
        response = self.llm.invoke(prompt)
        return response.content

def main():
    chat_agent = QA_Agent()
    img = "/workspaces/PnP_Pl/colcon_ws/src/LLM_Control/llm_image_analyser/llm_image_analyser/kuka_img.png"
    response = chat_agent.agent_chat("Turn 180 degrees and rise 2 units", img)
    print(f"Assistant: {response}")
    # while True:
    #     prompt = input("Enter you query|('/exit' to quit session): ")
    #     if prompt == "/exit":
    #         print("You will have a fortuitous encounter soon, God Speed!")
    #         break
    #     response = chat_agent.agent_chat(prompt, img)
    #     print(f"Assistant: {response}")
    #     print("-"*30)
    # chat_agent.save_history()

if __name__ == "__main__":
    main()