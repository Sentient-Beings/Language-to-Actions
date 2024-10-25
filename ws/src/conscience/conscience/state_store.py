import redis
import json
import time
from typing import Dict, Any, Optional

class StateStore:
    """Manages state storage and retrieval using Redis"""
    
    def __init__(self):
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
        
    def store_graph_state(self, state_type: str, state_data: Any):
        """Store a graph state update"""
        try:
            # Ensure state_data is a string or dict before storing
            if isinstance(state_data, (dict, list)):
                state_data = json.dumps(state_data)
            elif not isinstance(state_data, str):
                state_data = json.dumps({"content": str(state_data)})
            else:
                # If it's a string, wrap it in a dict for consistent format
                state_data = json.dumps({"content": state_data})
                
            self.redis_client.set(f"graph_state:{state_type}", state_data)
            # here we store timestamp for ordering
            self.redis_client.zadd("state_updates", {state_type: float(time.time())})
        except Exception as e:
            print(f"Error storing state: {e}")
            
    def get_graph_state(self, state_type: str) -> Optional[Dict]:
        """Retrieve a graph state"""
        try:
            data = self.redis_client.get(f"graph_state:{state_type}")
            if data:
                parsed_data = json.loads(data)
                # If it's a dict with content key, return just the content
                if isinstance(parsed_data, dict) and "content" in parsed_data:
                    return parsed_data["content"]
                return parsed_data
            return None
        except json.JSONDecodeError as e:
            print(f"Error retrieving state: {e}")
            return None
        except Exception as e:
            print(f"Error retrieving state: {e}")
            return None
            
    def get_all_states(self) -> Dict[str, Any]:
        """Get all current states"""
        states = {}
        try:
            # here we get ordered state types
            state_types = self.redis_client.zrange("state_updates", 0, -1)
            
            for state_type in state_types:
                state_type = state_type.decode('utf-8')
                state_data = self.get_graph_state(state_type)
                if state_data:
                    states[state_type] = state_data
            return states
        except Exception as e:
            print(f"Error retrieving all states: {e}")
            return {}
