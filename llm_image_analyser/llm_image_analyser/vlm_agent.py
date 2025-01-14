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
        Convert PIL images to Base64 encoded strings.

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
            "You are a KUKA robot capable of perceiving and interacting with your environment. "
            "You have sensors that allow you to see and understand the objects and obstacles "
            "around you, and you can take actions such as moving, picking objects, and navigating "
            "to specific locations. You are currently viewing a live image stream of your "
            "surroundings. Based on this image, you need to make decisions about what actions to "
            "take."
            ""
            "Your immediate task is to interact with the environment based on user's requirement "
            "and what you see in the image "
            "This could involve moving toward objects, avoiding obstacles, picking up "
            "or manipulating objects, or navigating to specific locations. If you identify an "
            "object or obstacle in your environment, respond with appropriate actions such as "
            "moving around it or interacting with it."
            ""
            "To perform the commands by the user you have access to below functions:"
            "move_lin: This function displaces your end effector in either x, y or z direction."
            "x is for left and right motion, y is for front and back motion back, finally "
            "z is for up and down. You only specify how much you want to move instead of goal "
            "pose. "
            "move_rot: This function rotates the end effector either about roll, pitch or yaw."
            "The units to be used for this function are degrees."
            "toggle_gripper: To toggle the gripper return the command toggle_gripper which takes a "
            "parameter to close the gripper in percentage. 0 means fully open and 99 means fully "
            "closed gripper."
        )
        return system_prompt

    def agent_chat(self, usr_prompt, img_path=None):
        image = self.convert_to_base64(img_path)
        input = [
            {"type": "image_url", "image_url": f"data:image/png;base64,{image}"},
            {"type": "text", "text": usr_prompt}
        ]
        prompt = [
            SystemMessage(self.get_system_prompt()),
            HumanMessage(input)
        ]
        response = self.llm.invoke(prompt)
        return response.content


def main():
    chat_agent = QA_Agent()
    img = "/workspaces/PnP_Pl/colcon_ws/src/LLM_Control/llm_image_analyser" + \
        "/llm_image_analyser/kuka_img.png"
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
