#### langgraph specific ####
from langgraph.graph import StateGraph, START, END
# from langchain_groq import ChatGroq
from langchain_openai import ChatOpenAI
from IPython.display import Image, display
from langgraph.graph import MessagesState
from langgraph.graph.message import add_messages
from langchain_core.messages import SystemMessage, HumanMessage, AIMessage, ToolMessage , AnyMessage , BaseMessage , RemoveMessage
from langchain.tools import BaseTool, StructuredTool, tool
#### general ####
from time import sleep
from typing_extensions import TypedDict
from operator import add
from pydantic import BaseModel , Field 
from uuid import uuid4
from typing import Literal, Dict, Any , Annotated , Optional, Type , Sequence , Callable
from dotenv import load_dotenv
from . import agent
import os
import logging
from . import state_store

## GLOBAL VARIABLES ##
_cached_graph = None
GROQ_API_KEY = os.getenv("GROQ_API_KEY")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
LANGSMITH_API_KEY = os.getenv("LANGSMITH_API_KEY")
os.environ["LANGCHAIN_PROJECT"] = "language-to-action"
os.environ["LANGSMITH_TRACING"] = "true"

## ERROR HANDLING ##
if not GROQ_API_KEY:
    raise ValueError("GROQ_API_KEY environment variable is not set")
if not OPENAI_API_KEY:
    raise ValueError("GROQ_API_KEY environment variable is not set")
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

#### observation data retrieval ####
SensorDataGetter = Callable[[], Dict[str, float]]
_get_sensor_data: SensorDataGetter = lambda: {}
#### user input retrieval ####
UserInputGetter = Callable[[], str]
_get_user_input: UserInputGetter = lambda: ""
# TODO: we need to remove this global variable passing approach, I have used it for quick testing 
control_command = "stop"

class User_Input_Retrieval():
    '''
    Encapsulate the user input retrieval logic
    '''     
    def setter_user_input(getter: UserInputGetter):
        global _get_user_input
        _get_user_input = getter
        
    def get_user_input(query: str = "") -> Dict[str, float]:
        data = _get_user_input()
        return data
    
class Observation_Retrieval():
    '''
    Encapsulate the observation data retrieval logic
    '''     
    def setter_observation_data(getter: SensorDataGetter):
        '''
        this function replaces the placeholder with the actual observation data getter that matches the actual implementation
        It is updated in the agent.py file
        '''
        global _get_observation_data
        _get_observation_data = getter
        
    def get_observation_data(query: str = "") -> str:
        '''
        A wrapper around the actual observation data getter
        '''
        data = _get_observation_data()
        return data

def define_model_with_tools() -> Type[ChatOpenAI]:
    """define the model with the tools that are required"""
    ### OpenAI Model
    model = ChatOpenAI(model="gpt-4o")
    
    ### Llama3 - 70B
    # model = ChatGroq(
    #     model="llama3-70b-8192",
    #     temperature=0,
    #     max_tokens=1000,
    #     request_timeout=60
    # )
    
    model_with_tools = model.bind_tools([robot_control_tool])
    return model_with_tools

# Graph State
class OverallState(TypedDict):
    """
    State of the agent
    """
    user_input: str
    observation: Optional[AnyMessage]
    previous_control_commands: Annotated[list[AnyMessage], add_messages]
    previous_decision_output: Optional[AnyMessage]
    pending_tool_calls: list[str]
    execution_data_summary: Optional[str]
    execution_ended: bool
    iterations: int

# Graph node
def get_observation(state: OverallState) -> OverallState:
    '''
    This function retrieves the scene observation data from the sensors and stores it in the state
    '''
    scene_observation = HumanMessage(content=Observation_Retrieval.get_observation_data(),id=str(uuid4()))
    state['observation'] = scene_observation
    # Store state update
    state_store.store_graph_state("observation", scene_observation.content)
    return state

# Graph node
def trim_control_data(state: OverallState) -> OverallState:
    '''
    We also store the last two control commands, just to imporve the LLMS's
    context and preventing it from repeating the same commands
    '''
    if len(state['previous_control_commands']) > 2:
        prune_control_commands = [RemoveMessage(id=m.id) for m in state['previous_control_commands'][:-2]]
        state['previous_control_commands'] = prune_control_commands
    return state

def format_control_actions_history(state: OverallState) -> str:
    """
    Formats the previous control commands history for clarity.
    This is done to improve the LLM's reasoning output
    """
    control_history = state.get('previous_control_commands')
    formatted_control_data = []

    for i, data in enumerate(control_history):
        if i == 0:
            time_label = "Control at t-1: "
        else:
            time_label = "Control at t-0: "

        formatted_control_data.append(f"{time_label}{data.content}")

    return "\n".join(formatted_control_data)

# TOOL 1 : Robot Control Tool
class SendControlCommands(BaseModel):
    '''
    Arg Schema: Controller tool that executes the high level control commands
    '''
    command: str = Field(default="stop", description="The command should be the direction in which you want the robot to move")

def high_level_control()-> str:
    '''
    This function returns the current control command which is used by the agent.py file to send the control commands to the robot
    '''
    return control_command

def send_control_commands(command: str) -> str:
    """Send the control Command to the robot"""
    global control_command 
    control_command = command
    agent.Agent_control.getter_high_level_command(high_level_control)
    if command == "stop":
        return "Halted the robot"
    else:
        return f"Moving the robot {command}"
    
robot_control_tool = StructuredTool.from_function(
    name="send_control_commands",
    func=send_control_commands,
    description="Use this tool to send the control commands to the robot, the control commands can either be forward, left, right, reverse or stop",
    args_schema= SendControlCommands
)

# Graph Node
def brain(state: OverallState) -> OverallState:
    """This function is the brain of the agent, it takes the observation and control history and reasons about the next action"""
    model_with_tools = define_model_with_tools()
    
    ## GUARD RAILS for Sensor
    if state.get('observation') is None or state['observation'].content == "":
        observation_info = "WARNING: No scene observations available. Safety protocol: Remain stationary."
    else:
        observation_info = "The scene in front of the robot:"

    ## GUARD RAILS for Controller
    if state.get('previous_control_commands'):
        try:
            control_info = format_control_actions_history(state)
        except Exception as e:
            control_info = "ERROR: Control history unavailable. Safety protocol: Issue STOP command."
    else:
        control_info = "No previous control commands. Initial movement requires some observation"

    messages = [
        SystemMessage(content="""You are controlling a differential drive robot with the following capabilities:
        Navigation Commands:
        - forward: Move straight ahead
        - left: Rotate left
        - right: Rotate right
        - stop: Halt all movement
        - reverse: Move backward

        Decision Making Protocol:
        1. Uncertainty Handling:
          - If scene description is not clear, default to STOP or reverse
          - Avoid rapid direction changes

        2. Planning:
          - Analyze the scene description and obstacles
          - Compare current scene with previous observations and decision outputs
          - Identify potential exploration paths

        3. Movement Strategy:
          - Donot repeat the same decisions too frequently
          - If unclear about the scene, default to STOP or reverse

        4. Exploration Rules:
          - Move to open spaces 
          - Be curious and explore new areas
        """),
        HumanMessage(content=f"""Mission Objective: {state['user_input']}
                     {observation_info} {state['observation']}
                     Control History: {control_info}
                     Decision History: {state['previous_decision_output']}""")
        ]

    response = model_with_tools.invoke(messages)
    state['previous_decision_output'] = [HumanMessage(content=response.content, id=str(uuid4()))]
    # Store state update
    state_store.store_graph_state("previous_decision_output", response.content)
    state['pending_tool_calls'] = response.tool_calls if isinstance(response.tool_calls, list) else [response.tool_calls]
    return state

# Graph Node
def execute_tools(state: OverallState) -> OverallState:
    """This function executes the tools that are pending"""
    state['iterations'] += 1
    tool_calls = state["pending_tool_calls"]
    for tool_call in tool_calls:
        if isinstance(tool_call, dict):
            if tool_call["name"].lower() == "send_control_commands":
                result = robot_control_tool.invoke(tool_call["args"])
                state["previous_control_commands"] = [HumanMessage(content=result, id=str(uuid4()))]
                # Store state update
                state_store.store_graph_state("previous_control_commands", result)
    return state

# Graph Node
def pause(state: OverallState) -> OverallState:
    """This function pauses the execution for 2 seconds, to prevent the excessive API calls to the model"""
    sleep(2) 
    return state

# Graph Node
def router(state: OverallState) -> Literal["END", "buffer"]:
    """This function routes the execution to the appropriate node based on the state of the execution
        It also checks for the user input and the number of iterations to decide the next node
    """
    user_input = User_Input_Retrieval.get_user_input()
    
    if state['iterations'] == 100:
        return "END"
    if user_input.lower() == "stop":
        return "END"
    else:
        return "buffer"
    
# Graph Builder
def build_graph(graph_builder:StateGraph) -> StateGraph:
    """This function builds the graph"""
    graph_builder.add_node("Scene observation", get_observation)
    graph_builder.add_node("Prune Control History", trim_control_data)
    graph_builder.add_node("brain", brain)

    graph_builder.add_edge(START, "Scene observation")
    graph_builder.add_edge("Scene observation", "Prune Control History")
    graph_builder.add_edge("Prune Control History", "brain")

    graph_builder.add_node("Controller", execute_tools)
    graph_builder.add_edge("brain", "Controller")

    graph_builder.add_node("buffer", pause)
    graph_builder.add_edge("buffer", "Scene observation")


    graph_builder.add_conditional_edges(
        "Controller",
        router,
        {"END": END, "buffer": "buffer"}
    )

    graph_builder.set_entry_point("Scene observation")
    graph = graph_builder.compile()
    return graph

# Graph Setup
def setup_graph():
    """This function sets up the graph"""
    global _cached_graph
    if _cached_graph is None:
        graph_builder = StateGraph(OverallState)
        _cached_graph = build_graph(graph_builder)
    return _cached_graph

# Graph Runner
def run_graph(user_message: str) -> Dict[str, Any]:
    """This function executes the graph"""
    initial_state = OverallState({
        "user_input": user_message,
        "observation": None,
        "previous_control_commands": [],
        "previous_decision_output": None,
        "pending_tool_calls": [],
        "execution_data_summary": "",
        "iterations" : 0
    })
    graph = setup_graph()
    actions = graph.invoke(initial_state, {"recursion_limit":1000})
    return actions
# Initialize state store
state_store = state_store.StateStore()
