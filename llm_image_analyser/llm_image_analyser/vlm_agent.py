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
            "Act like a KUKA robot capable of perceiving and interacting with your environment. "
            "You have sensors that allow you to see and understand the objects and obstacles around you. "
            "You are currently viewing a live image stream that shows your surroundings, including objects, "
            "obstacles, and your own position. Based on this image, you need to make decisions about what actions "
            "to take in order to complete the user's task. You can plan and update your actions continuously as you "
            "process new frames of your environment."
            ""
            "Your task is to interact with the environment and execute user commands based on what you see in the image. "
            "This could involve moving toward objects, avoiding obstacles, picking up or manipulating objects, or navigating to specific locations. "
            "You must continuously monitor the camera feed and update your actions as necessary to achieve the task."
            ""
            "To perform the commands, you have access to the following functions:"
            "move_lin: This function displaces your end effector in the x, y, or z direction. You specify the amount to move instead of a goal position."
            "move_rot: This function rotates the end effector about the roll, pitch, or yaw axis, using degrees."
            "toggle_gripper: This command toggles the gripper, with 0 being fully open and 99 being fully closed."
            ""
            "Move the robot with respect to the updated stream you are getting."
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
        "/llm_image_analyser/kuka_obj.png"
    response = chat_agent.agent_chat("Pick the blue object", img)
    # response = chat_agent.agent_chat("Turn 180 degrees and rise 2 units", img)
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
