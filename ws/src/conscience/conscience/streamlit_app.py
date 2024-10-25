import streamlit as st
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
from conscience.state_store import StateStore
import time

class ROSPublisher(Node):
    def __init__(self):
        super().__init__('streamlit_publisher')
        self.publisher = self.create_publisher(String, 'user_input', 10)
        
    def publish_message(self, message: str):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {message}')

class StreamlitApp:
    def __init__(self):
        try:
            rclpy.init()
        except RuntimeError:
            pass
            
        self.ros_publisher = ROSPublisher()
        self.ros_thread = threading.Thread(target=self._spin_ros)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        self.state_store = StateStore()
        
    def _spin_ros(self):
        rclpy.spin(self.ros_publisher)
        
    def run(self):
        st.title("Robot Control Interface")
        
        user_input = st.text_input("Enter your command:", key="user_input")
        if st.button("Send"):
            if user_input:
                self.ros_publisher.publish_message(user_input)
                
                if 'messages' not in st.session_state:
                    st.session_state.messages = []
                st.session_state.messages.append({"role": "user", "content": user_input})
                
        if 'messages' in st.session_state:
            for message in st.session_state.messages:
                with st.chat_message(message["role"]):
                    st.write(message["content"])
                    
        col1, col2, col3 = st.columns(3)
        
        with col1:
            st.subheader("Scene Observation")
            scene_state = self.state_store.get_graph_state("observation")
            if scene_state:
                st.write(scene_state)
                
        with col2:
            st.subheader("Agent Reasoning")
            reasoning_state = self.state_store.get_graph_state("previous_decision_output")
            if reasoning_state:
                st.write(reasoning_state)
                
        with col3:
            st.subheader("Final Decision")
            control_state = self.state_store.get_graph_state("previous_control_commands")
            if control_state:
                st.write(control_state)
                
        time.sleep(1)
        st.rerun()

    def __del__(self):
        """Cleanup when the app is closed"""
        try:
            rclpy.shutdown()
        except Exception:
            pass

@st.cache_resource
def get_streamlit_app():
    """Create a singleton instance of StreamlitApp"""
    return StreamlitApp()

def main():
    app = get_streamlit_app()
    app.run()

if __name__ == "__main__":
    main()
