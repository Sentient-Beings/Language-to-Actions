from time import sleep
import time
from typing_extensions import TypedDict
from langgraph.graph import StateGraph, START, END
# from langchain_groq import ChatGroq
from langchain_openai import ChatOpenAI
from IPython.display import Image, display
from langgraph.graph import MessagesState
from langgraph.graph.message import add_messages
from operator import add
from langchain.tools import BaseTool, StructuredTool, tool
from pydantic import BaseModel , Field 
from langchain_core.messages import SystemMessage, HumanMessage, AIMessage, ToolMessage , AnyMessage , BaseMessage , RemoveMessage
from uuid import uuid4
from typing import Literal, Dict, Any , Annotated , Optional, Type , Sequence , Callable
# from .agent import Agent_control
from . import agent
import asyncio
import json 
import random
import getpass
import os
import logging

# GLOBAL VARIABLES 
_cached_graph = None
GROQ_API_KEY = os.getenv("GROQ_API_KEY")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
LANGSMITH_API_KEY = os.getenv("LANGSMITH_API_KEY")
LANGCHAIN_PROJECT = os.getenv("LANGCHAIN_PROJECT")
if not GROQ_API_KEY:
    raise ValueError("GROQ_API_KEY environment variable is not set")
if not OPENAI_API_KEY:
    raise ValueError("GROQ_API_KEY environment variable is not set")
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
# Setup the Lidar Retrieval 
SensorDataGetter = Callable[[], Dict[str, float]]
_get_sensor_data: SensorDataGetter = lambda: {}
# Get the user input
UserInputGetter = Callable[[], str]
_get_user_input: UserInputGetter = lambda: ""
# define the control command
control_command = "stop"

#Define the State Blueprint 
class OverallState(TypedDict):
    """
    This state of the agent should be updated as we receive input from the user or sensor data.
    """
    user_input: str
    # messages: Annotated[list[dict], add]
    lidar_data_history: Annotated[list[AnyMessage], add_messages]
    last_control_output: Annotated[list[AnyMessage], add_messages]
    pending_tool_calls: list[str]
    execution_summary: Optional[str]
    execution_ended: bool
    iterations: int

class User_Input_Retrieval():
    '''
    encapsulate the sensor data retrieval logic
    '''     
    def setter_user_input(getter: SensorDataGetter):
        global _get_user_input
        _get_user_input = getter
        
    def get_user_input(query: str = "") -> Dict[str, float]:
        data = _get_user_input()
        return data
class Lidar_retrieval():
    '''
    encapsulate the sensor data retrieval logic
    '''     
    def setter_sensor_data(getter: SensorDataGetter):
        '''
        this function replaces the placeholder with the actual sensor data getter that matches the actual implementation
        It is updated in the agent.py file
        '''
        global _get_sensor_data
        _get_sensor_data = getter
        
    def get_sensor_data(query: str = "") -> Dict[str, float]:
        '''
        a wrapper around the actual sensor data getter
        '''
        data = _get_sensor_data()
        return data
# Graph node : Request Sensor Data
def update_sensor_data(state: OverallState) -> OverallState:
    sensor_data = ToolMessage(content=str(Lidar_retrieval.get_sensor_data()), name= "Lidar Sensor Data", tool_call_id=str(uuid4()))
    state['lidar_data_history'] = sensor_data
    return state

# Graph Node: Trim the Lidar Sensor to only keep the last two sensor data
def trim_sensor_data(state: OverallState) -> OverallState:
    if len(state['lidar_data_history']) >2:
        delete_some_sensor_data = [RemoveMessage(id=m.id) for m in state['lidar_data_history'][:-2]]
        state['lidar_data_history'] = delete_some_sensor_data
    return state

# Graph node to trim the control data to only keep the last two control data
def trim_control_data(state: OverallState) -> OverallState:
    if len(state['last_control_output']) > 3:
        delete_some_control_data = [RemoveMessage(id=m.id) for m in state['last_control_output'][:-3]]
        state['last_control_output'] = delete_some_control_data
    return state

def format_lidar_data_history(state: OverallState) -> str:
    lidar_data_history = state.get('lidar_data_history')
    formatted_data = []

    for i, data in enumerate(lidar_data_history):
        if i == 0:
            time_label = "Previous (t-1): "
        else:
            time_label = "Current (t-0): "
        
        formatted_data.append(f"{time_label}{data.content}")

    return "\n".join(formatted_data)

def format_control_actions_history(state: OverallState) -> str:
    last_control_output = state.get('last_control_output')
    formatted_data = []

    for i, data in enumerate(last_control_output):
        if i == 0:
            time_label = " Command Before Last: "
        elif i == 1:
            time_label = " Last Command: "
        else:
            time_label = " Current Command: "
        
        formatted_data.append(f"{time_label}{data.content}")

    return "\n".join(formatted_data)

# TOOL 1 : Robot Control Tool
class SendControlCommands(BaseModel):
    '''
    Arg Schema for the controller tool that interprets the high level control commands and send the commands to a low level controller
    '''
    query: str = Field(default="stop", description="the input should be the direction in which you want the robot to move into") 

def high_level_control()-> str:
    return control_command

async def a_high_level_control(direction:str)-> str:
    agent.Agent_control.getter_high_level_command(direction)
    return f"Moving the robot in the {direction} direction"

def send_control_commands(query: str) -> str:
    global control_command
    control_command = query
    agent.Agent_control.getter_high_level_command(high_level_control)
    return f"Moving the robot in the {query} direction"

async def asend_control_commands(query: str) -> str:
    response = await a_high_level_control(query)
    return response
    
robot_control_tool = StructuredTool.from_function(
    name="send_control_commands",
    func=send_control_commands,
    description="Use this tool to send the movement commands to the robot, the movement commands can only be forward, left, right or stop",
    args_schema= SendControlCommands,
    coroutine= asend_control_commands
)
# TOOL 2 : End Execution Tool
class EndExecution(BaseModel):
    '''
    No arguments are needed to call the End Execution tool. This tool stops the movement of the robot. 
    '''
    query: str = Field(default="", description="No arguments need to be provided to execute this Tool") 

def end_execution(query: str) -> str:
    response = high_level_control("stop")
    return response
    
end_exec_tool = StructuredTool.from_function(
    name="end_execution",
    func=end_execution,
    description="Use this tool to end the execution of the robot",
    args_schema= EndExecution,
)

def define_model_with_tools() -> Type[ChatOpenAI]:
    '''
        define the model with the tools that are required
        we have lidar sensor and robot control tool 
    '''
    model = ChatOpenAI(model="gpt-4o")
                       
    # model = ChatGroq(
    #     model="llama3-70b-8192",
    #     temperature=0,
    #     max_tokens=1000,
    #     request_timeout=60
    # )
    model_with_tools = model.bind_tools([robot_control_tool, end_exec_tool])
    return model_with_tools


def brain(state: OverallState) -> OverallState:
    model_with_tools = define_model_with_tools()
    
    ## GUARD RAILS for Sensor
    if len(state.get('lidar_data_history')) == 0:
        sensor_info = "We donot have any sensor data ! Be Cautious and do not move"
    else:
        sensor_info = "This is the history of lidar data, take this into account for making control decison "
        
    ## GUARD RAILS for Controller
    if state.get('last_control_output'):
        try:
            formatted_control_actions = format_control_actions_history(state)
            control_info = f"These are the last two control commands that you sent to the robot: {formatted_control_actions}"
        except Exception as e:
            control_info = "Error reading last control output. Send STOP COMMAND !"
    else:
        control_info = "There are no running control commands, we havent send any control commands, analyze the lidar data to decide what control commands you must send"
    
    lidar_data_history = format_lidar_data_history(state)
    # print(lidar_data_history)
    messages = [
        SystemMessage(content="""You are an AI Agent controlling a robot. 

        1. Analyze the lidar sensor readings which give the distance to obstacle in each direction, just try to avoid getting too close to an obstacle or hitting the obstacle. By too close i mean less than 0.5 meters.
        2. Be curious and move around, try to cover as much area as possible.
        3. you are given an history of commands that were sent to the robot. Do not change direction too frequently, try to maintain a smooth movement.
        4. It is okay to send the same command consecutively , if you think that is the right direction to move in.
        
        Also, if you observe from the lidar data that there is an obstacle too close to robot, then you should send the stop command to the robot.
        """),
        SystemMessage(content=f"{sensor_info}: {lidar_data_history}"),
        SystemMessage(content=f"{control_info}"),
        HumanMessage(content=f"User has asked to : {state['user_input']}")
    ]
    
    response = model_with_tools.invoke(messages)

    state['messages'] = [{"AI": AIMessage(content=response.content)}]
    # print(response.tool_calls)
    state['pending_tool_calls'] = response.tool_calls if isinstance(response.tool_calls, list) else [response.tool_calls]
    
    return state

# execute the tools 
def execute_tools(state: OverallState) -> OverallState:
    state['iterations'] += 1
    tool_calls = state["pending_tool_calls"]
    for tool_call in tool_calls:
        if isinstance(tool_call, dict):
            if tool_call["name"].lower() == "send_control_commands":
                result = robot_control_tool.invoke(tool_call["args"])
                state["last_control_output"] = [result]
                # state["messages"].append({"Robot Controller Tool": result})
            elif tool_call["name"].lower() == "end_execution":
                output = end_exec_tool.invoke(tool_call["args"])
                # state["messages"].append({"end execution tool": output})
                state["execution_ended"] = True
    return state

def pause(state: OverallState) -> OverallState:
    # sleep for 3 seconds, to prevent the excessive API calls to the model
    sleep(2) 
    return state

def router(state: OverallState) -> Literal["END", "buffer"]:
    user_input = User_Input_Retrieval.get_user_input()
    print(f"In graph router, user input is: {user_input}")
    if state["execution_ended"]:
        return "END"
    if state['iterations'] == 3:
        return "END"
    if user_input.lower() == "stop":
        return "END"
    else:
        return "buffer"
    
    
def build_graph(graph_builder:StateGraph) -> StateGraph:
    '''
    build the graph 
    '''

    graph_builder.add_node("Lidar", update_sensor_data)
    graph_builder.add_node("Trim_lidar_history", trim_sensor_data)
    graph_builder.add_node("Trim_control_history", trim_control_data)
    graph_builder.add_node("brain", brain)
    
    graph_builder.add_edge(START, "Lidar")
    graph_builder.add_edge("Lidar", "Trim_lidar_history")
    graph_builder.add_edge("Trim_lidar_history", "Trim_control_history")
    graph_builder.add_edge("Trim_control_history", "brain")
    
    graph_builder.add_node("Controller", execute_tools)
    graph_builder.add_edge("brain", "Controller")
    
    graph_builder.add_node("buffer", pause)
    graph_builder.add_edge("buffer", "Lidar")

    # graph_builder.add_node("termination", termination)
    # graph_builder.add_edge("termination", END)

    graph_builder.add_conditional_edges(
        "Controller",
        router,
        {"END": END, "buffer": "buffer"}
    )
    
    graph_builder.set_entry_point("Lidar")
    graph = graph_builder.compile()
    return graph

def setup_graph():
    global _cached_graph
    if _cached_graph is None:
        graph_builder = StateGraph(OverallState)
        _cached_graph = build_graph(graph_builder)
    return _cached_graph

def run_graph(user_message: str) -> Dict[str, Any]:
    initial_state = OverallState({
        "user_input": user_message,
        "messages": [],
        "lidar_data_history": [], 
        "last_control_output": [],  
        "pending_tool_calls": [],
        "execution_summary": "",
        "execution_ended": False,
        "iterations" : 0,
    })
    graph = setup_graph()
    actions = graph.invoke(initial_state, {"recursion_limit":1000})
    
    return actions