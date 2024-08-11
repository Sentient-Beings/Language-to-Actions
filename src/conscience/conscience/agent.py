import rclpy
from rclpy.node import Node
from agent_interfaces.srv import GetSensorDistances
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
from IPython.display import Image, display

################################################################################################################
############################################## Model Definition ################################################
################################################################################################################
load_dotenv()
grok_key = os.getenv('GROQ_API_KEY')
model = ChatGroq(temperature=0) 

################################################################################################################
############################################## LangGraph functions #############################################
################################################################################################################
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
    description = "Obtains the sensor data, essential before making any decisions regarding the user's input, the tool does not require any arguments"

    x = "Agent"
    def __init__(self, Agent:"Agent"):
        self.Agent = Agent

    def _run(self, query: str= "") -> dict[str, float]:
        request = GetSensorDistances.Request()
        response = self.Agent.get_minimum_distance(request)
        response = {"min_distance_right": response.min_distance_right, "min_distance_left": response.min_distance_left, "min_distance_front": response.min_distance_front}
        return response
    async def run(self, query: str= "") -> dict[str, float]:
        return self._run(query)

def define_model() -> Type[ChatGroq]:
    '''
    define the model with the tools that are required
    For now, we only need the GetSensorDataTool
    '''
    model_with_tools = model.bind_tools([GetSensorDataTool])
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
    model_with_tools = define_model()
    messages = state['user_input']
    response = model_with_tools.invoke(messages)
    state['response'] = response
    return state

def select_tool_execute(self, state: State, tools: dict[str, BaseTool]) -> State:
    '''
    select the tool to be used and execute it
    '''
    messages = state['response']
    tool_call = messages.tool_calls[0]
    tool = tools[tool_call.name.lower()]
    tool_output = tool.invoke(tool_call["args"])
    tool_message = ToolMessage(content=tool_output, tool_call_id = tool_call["id"])
    state['tool_ouput'] = tool_message
    return state

def build_graph(self) -> StateGraph:
    '''
    build the graph 
    '''
    graph_builder = StateGraph(GetSensorDataTool)
    graph_builder.add_node("process_user_input", self.proceess_input)
    graph_builder.edge(START, "process_user_input")
    graph_builder.add_node("Invoke_LLM", self.LLM_Analyze)
    graph_builder.edge("process_user_input", "Invoke_LLM")
    graph_builder.add_node("select_tool_execute", self.select_tool_execute)
    graph_builder.edge("Invoke_LLM", "select_tool_execute")
    graph_builder.add_edge("select_tool_execute", END)
    graph = self.graph_builder.compile()
    return graph

def visualize_graph(self, graph: StateGraph):
    '''
    visualize the graph
    '''
    try:
        display(Image(graph.get_graph().draw_mermaid_png()))
    except Exception:
        # This requires some extra dependencies and is optional
        pass

def execute_callback(self):
    '''
    execute the graph
    '''
    state = {
        "user_input": self.last_user_input,
        "sensor_data": {},
        "response": [],
        "tool_output": {}
    }
    graph = self.build_graph()
    result = graph.invoke(state)
    print(result)

####################################################################################################################
############################################## ROS2 COMM Class #####################################################
####################################################################################################################
class Agent(Node):
    def __init__(self):
        super().__init__('agent')

        self.declare_parameter('user_input', 'What is the nearest distance to the object?')
        self.last_user_input = self.get_parameter('user_input').value
        self.create_timer(1.0, self.check_user_input)
        self.sensor_client_ = self.create_client(GetSensorDistances, 'get_saftey_distance')
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
        
def main(args=None):
    rclpy.init(args=args)

    agent = Agent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()