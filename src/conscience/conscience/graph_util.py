import os
from dotenv import load_dotenv
from langgraph.graph import StateGraph, START, END
from typing import Annotated, Sequence , Dict , List  ,Callable
from typing_extensions import TypedDict
from langchain.tools import BaseTool, StructuredTool, tool
from langchain_core.messages import HumanMessage, ToolMessage , SystemMessage , BaseMessage , AIMessage
from IPython.display import Image, display
from langchain.tools import Tool
from langchain_groq import ChatGroq
from typing import Optional, Type
from pydantic import BaseModel
import logging

GROQ_API_KEY = os.getenv("GROQ_API_KEY")
if not GROQ_API_KEY:
    raise ValueError("GROQ_API_KEY environment variable is not set")
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class State(TypedDict):
    """
    This state of the agent should be updated as we receive input from the user or sensor data.
    """
    user_input: str
    sensor_data: dict[str, float]
    response: Sequence[BaseMessage]
    tool_output: dict[str, float]
    final_answer: Optional[str]

def process_input(state: State) -> State:
    '''
    process the input from the user
    '''
    logger.info(f"Processing user input: {state['user_input']}")
    state['user_input'] = state['user_input']
    return state

class GetSensorDataInput(BaseModel):
    query: str = ""

# this is a placeholder for the sensor data getter 
SensorDataGetter = Callable[[], Dict[str, float]]
_get_sensor_data: SensorDataGetter = lambda: {}

def set_sensor_data_getter(getter: SensorDataGetter):
    '''
    this function replaces the placeholder with the actual sensor data getter
    that matches the actual implementation
    '''
    global _get_sensor_data
    _get_sensor_data = getter
    logger.info("Sensor data getter has been set")

def get_sensor_data(query: str = "") -> Dict[str, float]:
    '''
    a wrapper around the actual sensor data getter
    '''
    logger.debug("Retrieving sensor data")
    data = _get_sensor_data()
    logger.info(f"Sensor data retrieved: {data}")
    return data

sensor_data_tool = Tool(
    name="lidar_sensor_data",
    func=get_sensor_data,
    description="Obtains the sensor data, essential before making any decisions regarding the user's input. This tool does not require any specific input.",
    args_schema=GetSensorDataInput
)

def setup_tools():
    return BasicToolNode([sensor_data_tool])

def define_model() -> Type[ChatGroq]:
    '''
    define the model with the tools that are required
    For now, we only need the GetSensorDataTool
    '''
    logger.info("Defining ChatGroq model with tools")
    model = ChatGroq(model="llama3-70b-8192", temperature=0, api_key=GROQ_API_KEY)
    model_with_tools = model.bind_tools([sensor_data_tool])
    return model_with_tools

def LLM_Analyze(state: Dict) -> Dict:
    logger.info("Starting LLM analysis")
    model_with_tools = define_model()
    messages = [
        SystemMessage(content="""You are an AI assistant that can use tools to gather information. 
        When you need to get sensor data, use the get_sensor_data tool. 
        This tool doesn't require any input, so you can call it with an empty string.
        Always use the tool when asked about sensor data."""),
        HumanMessage(content=state['user_input'])
    ]
    response = model_with_tools.invoke(messages)
    state['response'] = [response]
    logger.info("LLM analysis completed")
    return state

class BasicToolNode:
    '''
    This class actually executes the tool and returns the output
    '''
    def __init__(self, tools: List[Tool]) -> None:
        '''
        map the tool name to the tool object
        '''
        self.tools_by_name = {tool.name: tool for tool in tools}
        logger.info(f"BasicToolNode initialized with tools: {list(self.tools_by_name.keys())}")

    def __call__(self, state: Dict) -> Dict:
        '''
        execute the tools and update the state
        '''
        logger.info("Executing tools")
        messages = state.get('response', [])
        if not messages or not isinstance(messages[-1], AIMessage):
            logger.warning("No AI message found in state")
            return state

        ai_message = messages[-1]
        tool_calls = ai_message.additional_kwargs.get('tool_calls', [])
        '''
        iterate over the tool calls and execute the tools
        For each tool call:
        
        (1) finds the corresponding tool by name.
        (2) Executes the tool (in this case, always with an empty query).
        (3) Creates a ToolMessage with the tool's output.
        (4) Adds the tool output to the state and appends the ToolMessage to the response.
        '''
        for tool_call in tool_calls:
            tool_name = tool_call.get('function', {}).get('name')
            if tool_name not in self.tools_by_name:
                logger.warning(f"Tool {tool_name} not found")
                continue

            tool = self.tools_by_name[tool_name]
            
            try:
                logger.info(f"Invoking tool: {tool_name}")
                tool_output = tool.invoke({"query": ""})
                tool_message = ToolMessage(
                    content=str(tool_output),
                    tool_call_id=tool_call.get('id', '')
                )
                state['tool_output'] = tool_message
                state['response'].append(tool_message)
                logger.info(f"Tool {tool_name} executed successfully")
            except Exception as e:
                logger.error(f"Error executing tool {tool_name}: {str(e)}")
                state['error'] = f"Error executing tool {tool_name}: {str(e)}"

        return state

def final_response(state: Dict) -> Dict:
    logger.info("Generating final response")
    
    # Create a new ChatGroq instance without tools
    model_without_tools = ChatGroq(
        model="llama3-70b-8192",
        temperature=0,
        max_tokens=1000,
        request_timeout=60,
        api_key=GROQ_API_KEY
    )
    
    # Extract the tool output from the state
    tool_output = next((msg.content for msg in state['response'] if isinstance(msg, ToolMessage)), "No sensor data available")
    
    messages = [
        SystemMessage(content=f"""You are an AI assistant tasked with interpreting sensor data. 
        The user's question is: "{state['user_input']}"
        You have received sensor data and need to provide a concise answer based on this data.
        Use only the provided sensor data to answer the question."""),
        HumanMessage(content=f"The sensor readings are: {tool_output}")
    ]
    
    response = model_without_tools.invoke(messages)
    state['response'].append(response)
    state['final_answer'] = response.content
    logger.info(f"Generated final answer: {state['final_answer']}")
    return state

def build_graph(graph_builder:StateGraph, tool_node) -> StateGraph:
    '''
    build the graph 
    '''
    logger.info("Building graph")
    # graph_builder.add_node("process_user_input", process_input)
    # graph_builder.add_edge(START, "process_user_input")
    
    graph_builder.add_node("Invoke_LLM", LLM_Analyze)
    graph_builder.add_edge(START, "Invoke_LLM")
    # graph_builder.add_edge("process_user_input", "Invoke_LLM")
    
    graph_builder.add_node("execute_tools", tool_node)
    graph_builder.add_edge("Invoke_LLM", "execute_tools")
    
    graph_builder.add_node("final_response", final_response)
    graph_builder.add_edge("execute_tools", "final_response")
    
    graph_builder.add_edge("final_response", END)
    graph = graph_builder.compile()
    logger.info("Graph built and compiled")
    return graph

_cached_graph = None

def setup_graph():
    global _cached_graph
    if _cached_graph is None:
        logger.info("Setting up graph")
        graph_builder = StateGraph(State)
        tool_node = setup_tools()
        _cached_graph = build_graph(graph_builder, tool_node)
        logger.info("Graph setup completed")
    return _cached_graph

def run_graph(user_message: str):
    logger.info(f"Running graph with user message: {user_message}")
    initial_state = {
        "user_input": user_message,
        "sensor_data": {}, 
        "response": [],  
        "tool_output": {},
        "final_answer": None
    }
    graph = setup_graph()
    result = graph.invoke(initial_state)
    logger.info("Graph execution completed")
    return result