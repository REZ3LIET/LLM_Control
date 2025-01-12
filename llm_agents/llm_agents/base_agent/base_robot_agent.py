from langchain_core.tools import tool
from langchain_ollama import ChatOllama
from langchain_core.prompts import ChatPromptTemplate
from langchain.agents import AgentExecutor, create_tool_calling_agent


@tool
def heelo_world():
    """Prompts user to add new tools and returns True"""
    greeting = "Heelo, to give me more capabilities introduce me to new tools!"
    print(greeting)
    return True


class BaseRobotAgent:
    def __init__(self, ollama_model="qwen2.5-coder:3b", tools=[heelo_world]):
        llm_model = ChatOllama(
            model=ollama_model,
            temperature=0
        )

        sys_prompt = self.get_system_prompt()
        prompt = ChatPromptTemplate.from_messages(
            [
                ("system", sys_prompt),
                ("human", "{input}"),
                ("placeholder", "{agent_scratchpad}")
            ]
        )

        llm_tool = create_tool_calling_agent(llm_model, tools, prompt)
        self.robot_agent = AgentExecutor(agent=llm_tool, tools=tools, verbose=True)

    def get_system_prompt(self):
        system_prompt = (
            "Act as a wise assistant and guide the user with their task."
        )
        return system_prompt

    def prompt_llm(self, prompt):
        response = self.robot_agent.invoke(
            {
                "input": prompt
            }
        )
        return response


def main():
    robot_agent = BaseRobotAgent()
    while True:
        prompt = input("User (/exit to exit): ")
        if prompt == "/exit":
            print("Exiting...")
            break
        response = robot_agent.send_command(prompt)
        print(f"Agent: {response}")


if __name__ == "__main__":
    main()
