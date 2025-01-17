import rclpy
from llm_agents.base_agent.base_robot_agent import BaseRobotAgent
from llm_agents.kuka_agent.actions.kuka_actions import move_to_pose


class KukaAgent(BaseRobotAgent):
    def __init__(self):
        super().__init__(tools=[move_to_pose])

    def get_system_prompt(self):
        system_prompt = (
            "You are a KUKA Arm. User will provide you poses and you have to reach them."
            "If the result of action is True then respond as 'Action executed Successfully'"
            "If the result of action is False then respond as 'Action execution Failed'"
        )
        return system_prompt

    def send_command(self, usr_cmd):
        response = self.prompt_llm(usr_cmd)
        return response


def main(args=None):
    rclpy.init(args=args)
    robot_agent = KukaAgent()
    while True:
        prompt = input("User (/exit to exit): ")
        if prompt == "/exit":
            print("Exiting...")
            break
        response = robot_agent.send_command(prompt)
        print(f"Agent: {response}")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
