import os
from dotenv import load_dotenv
from langgraph.graph import StateGraph, START, END
from typing import Annotated, Sequence , Dict , List  ,Callable
from typing_extensions import TypedDict
from langgraph.graph import StateGraph, START, END
from langchain.tools import BaseTool, StructuredTool, tool
from langchain_core.messages import HumanMessage, ToolMessage , SystemMessage , BaseMessage , AIMessage
from IPython.display import Image, display
from langchain.tools import Tool
from langchain_groq import ChatGroq
from typing import Optional, Type
from pydantic import BaseModel
from typing import Annotated, Sequence , Dict , List
from langchain.tools import Tool

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

def get_sensor_data() -> Dict[str, float]:
    '''
    a wrapper around the actual sensor data getter
    '''
    return _get_sensor_data()

sensor_data_tool = Tool(
    name="get_sensor_data",
    func=get_sensor_data,
    description="Obtains the sensor data, essential before making any decisions regarding the user's input. This tool does not require any arguments.",
    args_schema=GetSensorDataInput
)

def setup_tools():
    return BasicToolNode([sensor_data_tool])

def define_model() -> Type[ChatGroq]:
    '''
    define the model with the tools that are required
    For now, we only need the GetSensorDataTool
    '''
    model = ChatGroq(model="llama3-8b-8192", temperature=0)
    model_with_tools = model.bind_tools([sensor_data_tool])
    return model_with_tools

def LLM_Analyze(state: Dict) -> Dict:
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
    return state

class BasicToolNode:
    '''
    This class actually executes the tool and returns the output
    '''
    def __init__(self, tools: List[Tool]) -> None:
        self.tools_by_name = {tool.name: tool for tool in tools}

    def __call__(self, state: Dict) -> Dict:
        messages = state.get('response', [])
        if not messages or not isinstance(messages[-1], AIMessage):
            return state

        ai_message = messages[-1]
        tool_calls = ai_message.additional_kwargs.get('tool_calls', [])

        for tool_call in tool_calls:
            tool_name = tool_call.get('function', {}).get('name')
            if tool_name not in self.tools_by_name:
                continue

            tool = self.tools_by_name[tool_name]
            
            try:
                tool_output = tool.invoke({"query": ""})
                tool_message = ToolMessage(
                    content=str(tool_output),
                    tool_call_id=tool_call.get('id', '')
                )
                state['tool_output'] = tool_message
                state['response'].append(tool_message)
            except Exception as e:
                print(f"Error executing tool {tool_name}: {str(e)}")
                state['error'] = f"Error executing tool {tool_name}: {str(e)}"

        return state

def final_response(state: Dict) -> Dict:
    print("Entering final_response")
    
    # Create a new ChatGroq instance without tools
    model_without_tools = ChatGroq(
        model="llama3-8b-8192",
        temperature=0,
        max_tokens=1000,
        request_timeout=60
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
    print(f"Generated final answer: {state['final_answer']}")
    print("Exiting final_response")
    return state

def build_graph(graph_builder:StateGraph, tool_node) -> StateGraph:
    '''
    build the graph 
    '''
    graph_builder.add_node("process_user_input", process_input)
    graph_builder.add_edge(START, "process_user_input")
    
    graph_builder.add_node("Invoke_LLM", LLM_Analyze)
    graph_builder.add_edge("process_user_input", "Invoke_LLM")
    
    graph_builder.add_node("execute_tools", tool_node)
    graph_builder.add_edge("Invoke_LLM", "execute_tools")
    
    graph_builder.add_node("final_response", final_response)
    graph_builder.add_edge("execute_tools", "final_response")
    
    graph_builder.add_edge("final_response", END)
    graph = graph_builder.compile()
    return graph

def setup_graph():
    graph_builder = StateGraph(State)
    tool_node = setup_tools()
    graph = build_graph(graph_builder, tool_node)
    return graph

def run_graph(user_message: str):
    initial_state = {
        "user_input": user_message,
        "sensor_data": {}, 
        "response": [],  
        "tool_output": {},
        "final_answer": None
    }
    graph = setup_graph()
    result = graph.invoke(initial_state)
    return result