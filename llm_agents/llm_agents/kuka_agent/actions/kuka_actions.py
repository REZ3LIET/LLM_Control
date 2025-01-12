from typing import List, Tuple
from langchain_core.tools import tool
from llm_agents.kuka_agent.actions.action_helper import KukaActionHelper


@tool
def move_to_pose(position_values: List[float]) -> Tuple[bool, str]:
    """
    Controls the Kuka Arm to reach the desired position values.
    The position values are to be in x, y, z, roll, pitch, yaw format.
    Returns True if action executed successfully else False along with
    an execution message.
    """
    print("Moving Kuka Arm")
    print(f"Destination: {position_values}")

    executer = KukaActionHelper()
    result = executer.move_xyzw(*position_values)
    return result
