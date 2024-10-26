import redis
import json
import time
from typing import Dict, Any, Optional
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class StateStore:
    """Manages state storage and retrieval using Redis"""
    
    def __init__(self):
        self.redis_client = redis.Redis(host='localhost', port=6379, db=0)
        
    def store_graph_state(self, state_type: str, state_data: Any):
        """Store a graph state update"""
        try:
            # Log before storing
            # logger.info(f"Storing state for {state_type}")
            # logger.debug(f"State data: {state_data}")
            
            if isinstance(state_data, (dict, list)):
                state_data = json.dumps(state_data)
            elif not isinstance(state_data, str):
                state_data = json.dumps({"content": str(state_data)})
            else:
                state_data = json.dumps({"content": state_data})
                
            # Store the state
            key = f"graph_state:{state_type}"
            self.redis_client.set(key, state_data)
            self.redis_client.zadd("state_updates", {state_type: float(time.time())})
            
            # Verify storage
            stored_data = self.redis_client.get(key)
            # logger.info(f"Stored state for {state_type}: {bool(stored_data)}")
            
        except Exception as e:
            logger.error(f"Error storing state: {e}")
            
    def get_graph_state(self, state_type: str) -> Optional[Dict]:
        """Retrieve a graph state"""
        try:
            # logger.info(f"Retrieving state for {state_type}")
            data = self.redis_client.get(f"graph_state:{state_type}")
            
            if not data:
                #logger.info(f"No data found for {state_type}")
                return None
                
            parsed_data = json.loads(data)
            if isinstance(parsed_data, dict) and "content" in parsed_data:
                # logger.info(f"Retrieved state for {state_type}")
                return parsed_data["content"]
            return parsed_data
            
        except Exception as e:
            logger.error(f"Error retrieving state for {state_type}: {e}")
            return None
            
    def get_all_states(self) -> Dict[str, Any]:
        """Get all current states"""
        states = {}
        try:
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

    def clear_all_states(self):
        """Clear all stored states from Redis"""
        try:
            keys = self.redis_client.keys("graph_state:*")
            if keys:
                self.redis_client.delete(*keys)
            self.redis_client.delete("state_updates")
            print("Cleared all states from Redis")
        except Exception as e:
            print(f"Error clearing states: {e}")

    def list_all_keys(self):
        """List all keys in Redis"""
        try:
            keys = self.redis_client.keys("graph_state:*")
            print("Current Redis Keys:")
            for key in keys:
                value = self.redis_client.get(key)
                print(f"{key.decode('utf-8')}: {value.decode('utf-8')}")
            return keys
        except Exception as e:
            print(f"Error listing keys: {e}")
            return []

    def check_db_size(self):
        """Check number of keys in Redis"""
        try:
            size = self.redis_client.dbsize()
            print(f"Current Redis DB size: {size} keys")
            return size
        except Exception as e:
            print(f"Error checking DB size: {e}")
            return 0
