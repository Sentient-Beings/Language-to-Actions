import rclpy
from rclpy.node import Node
from agent_interfaces.srv import GetSensorDistance
from std_msgs.msg import String
import os
from dotenv import load_dotenv
# LangGraph essential imports
from typing import Annotated, Sequence
from typing_extensions import TypedDict
from langgraph.graph import StateGraph, START, END
from langchain_groq import ChatGroq
from langchain.tools import BaseTool, StructuredTool, tool
from langchain_core.messages import HumanMessage, ToolMessage 
from langchain_core.messages import BaseMessage
from typing import Optional, Type

class State(TypedDict):
    """
    This state of the agent should be updated as we receive input from the user or sensor data.
    """
    user_input: str
    sensor_data: dict[str, float]
    response: Sequence[BaseMessage]
    tool_output: dict[str, float]

class GetSensorDataTool(BaseTool):
    name = "get_sensor_data"
    description = "Obtains the sensor data, essential before making any decisions regarding the user's input"

    x = "Agent"
    def __init__(self, Agent:"Agent"):
        self.Agent = Agent

    def _run(self, query: str= "") -> dict[str, float]:
        request = GetSensorDistance.Request()
        response = self.Agent.get_minimum_distance(request)
        return response
    async def run(self, query: str= "") -> dict[str, float]:
        return self._run(query)

class Agent(Node):
    def __init__(self, model: ChatGroq):
        super().__init__('agent')
        self.model = model
        self.declare_parameter('user_input', 'What is the nearest distance to the object?')
        self.last_user_input = self.get_parameter('user_input').value
        self.create_timer(1.0, self.check_user_input)
        self.sensor_client_ = self.create_client(GetSensorDistance, 'get_saftey_distance')
        self.tool_mapping = {
            "get_sensor_data": GetSensorDataTool
        }
        self.get_logger().info('Agent initialized')
    
    def check_user_input(self):
        '''
        check if the user input has changed, if so, update the last_user_input
        '''
        current_user_input = self.get_parameter('user_input').value
        if current_user_input != self.last_user_input:
            self.last_user_input = current_user_input

    def get_minimum_distance(self, request):
        '''
        client function to get the minimum distance from the sensor
        '''
        while not self.sensor_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.sensor_client_.call_async(request) 
        future.add_done_callback(self.future_callback)

    def future_callback(self, future):
        try:
            response = future.result()
            return response
        except Exception as e:
            self.get_logger().info(f'Error: {e}')
    
    ### LangGraph functions
    def define_model(self) -> Type[ChatGroq]:
        '''
        define the model with the tools that are required
        For now, we only need the GetSensorDataTool
        '''
        model_with_tools = self.model.bind_tools([GetSensorDataTool])
        return model_with_tools
    
    def proceess_input(self, state: State) -> State:
        '''
        process the input from the user
        '''
        state['user_input'] = [HumanMessage(content=state['user_input'])]

    def LLM_Analyze(self, state: State, model: ChatGroq) -> State:
        '''
        Analyze the user input and make a decision
        '''
        messages = state['user_input']
        response = model.invoke(messages)
        state['response'] = response
        return state

    def select_tool_execute(self, state: State, tools: dict[str, BaseTool]) -> State:
        '''
        select the tool to be used and execute it
        '''
        message = state['response']
        tool_call = message.tool_calls[0]
        tool = tools[tool_call.name.lower()]
        tool_output = tool.run(tool_call.arguments)
        state['tool_ouput'] = tool_output
        return state
        
def main(args=None):
    rclpy.init(args=args)

    load_dotenv()
    grok_key = os.getenv('GROQ_API_KEY')
    model = ChatGroq(temperature=0) 

    agent = Agent(model)
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()