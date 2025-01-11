from typing import List
from langchain_core.tools import tool
from llm_agents.kuka_agent.actions.action_helper import KukaActionHelper

@tool
def move_to_pose(position_values: List[float]) -> bool:
    """
    Controls the Kuka Arm to reach the desired position values.
    Returns True if action executed successfully else False.
    """
    print("Moving Kuka Arm")
    print(f"Destination: {position_values}")

    executer = KukaActionHelper()
    result = executer.move_xyzw(*position_values)
    return result
