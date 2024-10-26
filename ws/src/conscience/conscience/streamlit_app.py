import streamlit as st
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
from conscience.state_store import StateStore
import time
import os
import base64

class ROSPublisher(Node):
    """Setup the ROS publisher for the user input"""
    def __init__(self):
        super().__init__('streamlit_publisher')
        self.publisher = self.create_publisher(String, 'user_input', 10)
        
    def publish_message(self, message: str):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)

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
        
        st.markdown("""
        <style>
        .update-number {
            display: inline-block;
            padding: 4px 8px;
            background-color: #f0f2f6;
            color: #31333F;
            border-radius: 16px;
            font-size: 14px;
            font-weight: bold;
            margin-bottom: 8px;
        }
        </style>
        """, unsafe_allow_html=True)

        st.markdown("""
        <style>
        .dot-wave {
            display: inline-block;
            height: 30px;
            width: 60px;
            position: relative;
        }
        .dot-wave::before,
        .dot-wave::after,
        .dot-wave span {
            content: "";
            display: inline-block;
            width: 10px;
            height: 10px;
            border-radius: 50%;
            background-color: #3498db;
            position: absolute;
            top: 10px;
            animation: wave 1.3s linear infinite;
        }
        .dot-wave::before {
            left: 0;
        }
        .dot-wave span {
            left: 20px;
            animation-delay: -0.2s;
        }
        .dot-wave::after {
            left: 40px;
            animation-delay: -0.4s;
        }
        @keyframes wave {
            0%, 60%, 100% {
                transform: translateY(0);
            }
            30% {
                transform: translateY(-10px);
            }
        }
        </style>
        """, unsafe_allow_html=True)

        # session state variables
        if 'scene_state' not in st.session_state:
            st.session_state.scene_state = None
        if 'reasoning_state' not in st.session_state:
            st.session_state.reasoning_state = None
        if 'control_state' not in st.session_state:
            st.session_state.control_state = None
        if 'messages' not in st.session_state:
            st.session_state.messages = []
        if 'updating' not in st.session_state:
            st.session_state.updating = False
        if 'update_interval' not in st.session_state:
            st.session_state.update_interval = 5
        if 'update_count' not in st.session_state:
            st.session_state.update_count = 0
        if 'waiting_for_update' not in st.session_state:
            st.session_state.waiting_for_update = False
        if 'waiting_message_key' not in st.session_state:
            st.session_state.waiting_message_key = None

    def _spin_ros(self):
        rclpy.spin(self.ros_publisher)
        
    def run(self):
        st.markdown("""
        <style>
        .header-container {
            display: flex;
            align-items: center;
            gap: 20px;
        }
        .header-logo {
            width: 80px;
        }
        .header-title {
            font-size: 28px;
            font-weight: bold;
            margin: 0;
        }
        </style>
        """, unsafe_allow_html=True)

        logo_path = os.path.join("conscience", "logo_sentient_beings.png")
        header_html = f"""
        <div class="header-container">
            <img src="data:image/png;base64,{self.get_base64_of_image(logo_path)}" class="header-logo">
            <h1 class="header-title">Language to Action Space</h1>
        </div>
        """
        st.markdown(header_html, unsafe_allow_html=True)

        for message in st.session_state.messages:
            with st.chat_message(message["role"]):
                if message["role"] == "user":
                    st.write(message["content"])
                else:
                    self.display_agent_states(message["content"])

        user_input = st.chat_input("Welcome to the Language to Action Agent!")
        if user_input:
            st.session_state.update_count = 0
            
            st.session_state.messages.append({"role": "user", "content": user_input})
            with st.chat_message("user"):
                st.write(user_input)

            self.ros_publisher.publish_message(user_input)
            
            if user_input.lower() == "stop":
                st.session_state.updating = False
                st.session_state.waiting_for_update = False
            else:
                st.session_state.updating = True
                st.session_state.waiting_for_update = True
                # with st.chat_message("assistant"):
                #     st.session_state.waiting_message_key = st.empty()
                #     st.session_state.waiting_message_key.markdown('<div class="dot-wave"><span></span></div>', unsafe_allow_html=True)
                    
        if st.session_state.updating:
            self.update_and_display_agent_response()

        time.sleep(st.session_state.update_interval)
        st.rerun()

    def update_and_display_agent_response(self):
        self.update_states()
        st.session_state.update_count += 1
        agent_response = {
            "update_number": st.session_state.update_count,
            "scene_state": st.session_state.scene_state,
            "reasoning_state": st.session_state.reasoning_state,
            "control_state": st.session_state.control_state,
        }
        
        # Remove the waiting animation, if it exists
        if st.session_state.waiting_message_key:
            st.session_state.waiting_message_key.empty()
        
        st.session_state.messages.append({
            "role": "assistant", 
            "content": agent_response
        })
        
        with st.chat_message("assistant"):
            self.display_agent_states(agent_response)
        
        st.session_state.waiting_for_update = True
        with st.chat_message("assistant"):
            st.session_state.waiting_message_key = st.empty()
            st.session_state.waiting_message_key.markdown('<div class="dot-wave"><span></span></div>', unsafe_allow_html=True)
            
    def update_states(self):
        """Update the states from the state store"""
        st.session_state.scene_state = self.state_store.get_graph_state("observation")
        st.session_state.reasoning_state = self.state_store.get_graph_state("previous_decision_output")
        st.session_state.control_state = self.state_store.get_graph_state("previous_control_commands")

    def display_agent_states(self, states):
        """Display the agent states in the UI"""
        if "update_number" in states:
            st.markdown(f'<div class="update-number">Update #{states["update_number"]}</div>', unsafe_allow_html=True)
        
        with st.expander("Scene Observation", expanded=False):
            if states["scene_state"]:
                st.write(states["scene_state"])
            else:
                st.write("No scene observation data available.")

        with st.expander("Agent Reasoning", expanded=False):
            if states["reasoning_state"]:
                st.write(states["reasoning_state"])
            else:
                st.write("No agent reasoning data available.")

        with st.expander("Final Decision", expanded=False):
            if states["control_state"]:
                st.write(states["control_state"])
            else:
                st.write("No final decision data available.")

    def __del__(self):
        """Cleanup when the app is closed"""
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def get_base64_of_image(self, image_path):
        with open(image_path, "rb") as img_file:
            return base64.b64encode(img_file.read()).decode('utf-8')

@st.cache_resource
def get_streamlit_app():
    return StreamlitApp()

def main():
    app = get_streamlit_app()
    app.run()

if __name__ == "__main__":
    main()
