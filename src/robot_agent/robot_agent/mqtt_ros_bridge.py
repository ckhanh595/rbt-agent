#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import paho.mqtt.client as mqtt
from paho.mqtt.client import CallbackAPIVersion
import threading

class MqttRosBridge(Node):
    def __init__(self):
        super().__init__('mqtt_ros_bridge')
        
        # Parameters (you can modify these for your setup)
        # This IP points to the host machine from the VM
        self.mqtt_broker = "10.0.2.2"  # Use the default NAT gateway for VirtualBox
        self.mqtt_port = 1883
        self.mqtt_client_id = "ros_robot_agent"
        
        # MQTT topics
        self.mqtt_cmd_topic = "robot/commands"    # Commands from Robot Manager to Robot
        self.mqtt_status_topic = "robot/status"   # Status updates from Robot to Robot Manager
        self.mqtt_telemetry_topic = "robot/telemetry"  # Telemetry data from Robot
        
        # ROS topics
        self.cmd_vel_topic = "cmd_vel"  # Standard topic for robot movement
        self.status_topic = "robot_status"
        self.telemetry_topic = "robot_telemetry"
        
        # Initialize ROS publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        
        # Subscribe to ROS topics to publish to MQTT
        self.telemetry_sub = self.create_subscription(
            String,
            self.telemetry_topic,
            self.telemetry_callback,
            10
        )
        self.status_sub = self.create_subscription(
            String,
            self.status_topic,
            self.status_callback,
            10
        )
        
        # Initialize MQTT client with explicit API version
        self.mqtt_client = mqtt.Client(
            client_id=self.mqtt_client_id, 
            transport="tcp",
            callback_api_version=CallbackAPIVersion.VERSION2
        )
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        # Connect to MQTT broker
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.get_logger().info(f"Connected to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")
            
            # Start MQTT loop in a separate thread
            self.mqtt_thread = threading.Thread(target=self.mqtt_client.loop_forever)
            self.mqtt_thread.daemon = True
            self.mqtt_thread.start()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {str(e)}")
        
        self.get_logger().info("MQTT-ROS Bridge initialized")
    
    def on_mqtt_connect(self, client, userdata, flags, reason_code, properties=None):
        self.get_logger().info(f"MQTT connected with result code {reason_code}")
        # Subscribe to command topic
        self.mqtt_client.subscribe(self.mqtt_cmd_topic)
   
    def on_mqtt_message(self, client, userdata, msg):
        try:
            # Parse the incoming message
            payload_str = msg.payload.decode('utf-8')
            self.get_logger().info(f"Received MQTT message on {msg.topic}: {payload_str}")
            
            # Process different message types
            if msg.topic == self.mqtt_cmd_topic:
                self.process_command(payload_str)
        except Exception as e:
            self.get_logger().error(f"Error processing MQTT message: {str(e)}")
    
    def process_command(self, payload_str):
        try:
            payload = json.loads(payload_str)
            command = payload.get('command', '')
            
            if command == 'move':
                # Extract movement parameters
                linear_x = float(payload.get('linear_x', 0.0))
                linear_y = float(payload.get('linear_y', 0.0))
                angular_z = float(payload.get('angular_z', 0.0))
                
                # Create and publish Twist message
                twist_msg = Twist()
                twist_msg.linear.x = linear_x
                twist_msg.linear.y = linear_y
                twist_msg.angular.z = angular_z
                self.cmd_vel_pub.publish(twist_msg)
                self.get_logger().info(f"Published movement command: linear=({linear_x}, {linear_y}), angular_z={angular_z}")
                
                # Publish status update
                self.publish_status(f"Executing move command: linear=({linear_x}, {linear_y}), angular_z={angular_z}")
            
            elif command == 'stop':
                # Stop the robot by publishing zero velocity
                twist_msg = Twist()
                self.cmd_vel_pub.publish(twist_msg)
                self.get_logger().info("Published stop command")
                
                # Publish status update
                self.publish_status("Robot stopped")
            
            elif command == 'getBattery':
                # Simulate battery status
                battery_level = 75  # simulated battery level
                self.publish_telemetry({"battery": battery_level})
                self.get_logger().info(f"Published battery status: {battery_level}%")
            
            elif command == 'getPosition':
                # Simulate position data
                position = {"x": 2.5, "y": 3.2, "theta": 0.75}
                self.publish_telemetry({"position": position})
                self.get_logger().info(f"Published position data: {position}")
            
            elif command == 'executeTask':
                task_id = payload.get('taskId', '')
                self.get_logger().info(f"Executing task: {task_id}")
                self.publish_status(f"Started task: {task_id}")
                
                # In a real implementation, this would trigger a task execution
                # For our POC, we'll just simulate a success message after a delay
                self.publish_status(f"Completed task: {task_id}")
            
            elif command == 'setParameter':
                param_name = payload.get('name', '')
                param_value = payload.get('value', '')
                self.get_logger().info(f"Setting parameter {param_name} to {param_value}")
                self.publish_status(f"Parameter {param_name} set to {param_value}")
            
            else:
                self.get_logger().warn(f"Unknown command: {command}")
        
        except Exception as e:
            self.get_logger().error(f"Error processing command: {str(e)}")
    
    def telemetry_callback(self, msg):
        # Forward telemetry from ROS to MQTT
        try:
            self.mqtt_client.publish(self.mqtt_telemetry_topic, msg.data)
            self.get_logger().debug(f"Published telemetry to MQTT: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error publishing telemetry to MQTT: {str(e)}")
    
    def status_callback(self, msg):
        # Forward status from ROS to MQTT
        try:
            self.mqtt_client.publish(self.mqtt_status_topic, msg.data)
            self.get_logger().debug(f"Published status to MQTT: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error publishing status to MQTT: {str(e)}")
    
    def publish_status(self, status_text):
        # Create status message for both ROS and MQTT
        status_msg = String()
        status_msg.data = json.dumps({"status": status_text, "timestamp": self.get_clock().now().to_msg().sec})
        
        # Publish to ROS topic
        self.status_pub.publish(status_msg)
        
        # Publish directly to MQTT
        self.mqtt_client.publish(self.mqtt_status_topic, status_msg.data)
    
    def publish_telemetry(self, telemetry_data):
        # Add timestamp to telemetry data
        telemetry_data["timestamp"] = self.get_clock().now().to_msg().sec
        
        # Create telemetry message
        telemetry_msg = String()
        telemetry_msg.data = json.dumps(telemetry_data)
        
        # Publish to MQTT directly
        self.mqtt_client.publish(self.mqtt_telemetry_topic, telemetry_msg.data)

def main(args=None):
    rclpy.init(args=args)
    bridge = MqttRosBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
