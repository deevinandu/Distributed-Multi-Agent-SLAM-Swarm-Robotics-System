import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import socket
import json
import math
import select

class UDPBridge(Node):
    def __init__(self):
        super().__init__('udp_bridge')
        
        # UDP Setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 8888))
        self.sock.setblocking(False)
        
        self.clients = {} # agent_id -> (ip, port)
        self.publishers_odom = {}
        self.publishers_scan = {}
        self.subscribers = {}
        
        # Check socket every 0.01s (100Hz)
        self.timer = self.create_timer(0.01, self.udp_callback)
        self.get_logger().info("UDP Bridge Started on Port 8888")

    def udp_callback(self):
        try:
            ready = select.select([self.sock], [], [], 0.0)
            if ready[0]:
                data, addr = self.sock.recvfrom(65535) # Max UDP size
                self.process_packet(data, addr)
        except Exception as e:
            self.get_logger().error(f"UDP Error: {e}")

    def process_packet(self, data, addr):
        try:
            msg = json.loads(data.decode('utf-8'))
            agent_id = msg.get('id')
            
            if not agent_id:
                return

            # Register new client
            if agent_id not in self.clients:
                self.clients[agent_id] = addr
                self.create_agent_interfaces(agent_id)
                self.get_logger().info(f"New Agent Connected: {agent_id} at {addr}")
            
            # Update address in case it changed (e.g. restart)
            self.clients[agent_id] = addr

            # 1. Publish Odometry
            if 'odom' in msg:
                self.publish_odom(agent_id, msg['odom'])

            # 2. Publish Scan
            if 'scan' in msg:
                self.publish_scan(agent_id, msg['scan'])
                
        except json.JSONDecodeError:
            self.get_logger().warn("Received invalid JSON")
        except Exception as e:
            self.get_logger().error(f"Packet Processing Error: {e}")

    def create_agent_interfaces(self, agent_id):
        # Publishers
        self.publishers_odom[agent_id] = self.create_publisher(
            Odometry, 
            f'/{agent_id}/odom', 
            10
        )
        self.publishers_scan[agent_id] = self.create_publisher(
            LaserScan, 
            f'/{agent_id}/scan', 
            10
        )
        
        # Subscriber for Cmd Vel
        self.subscribers[agent_id] = self.create_subscription(
            Twist,
            f'/{agent_id}/cmd_vel',
            lambda m, aid=agent_id: self.cmd_vel_callback(m, aid),
            10
        )

    def publish_odom(self, agent_id, data):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        
        msg.pose.pose.position.x = float(data.get('x', 0))
        msg.pose.pose.position.y = float(data.get('y', 0))
        
        yaw = float(data.get('yaw', 0))
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.publishers_odom[agent_id].publish(msg)

    def publish_scan(self, agent_id, ranges):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_link"
        
        # Match main.cpp params
        msg.angle_min = -1.57
        msg.angle_max = 1.57
        msg.angle_increment = 3.14 / 180.0
        msg.range_min = 0.05
        msg.range_max = 2.0
        msg.scan_time = 2.0 # 2 seconds per scan
        
        msg.ranges = [float(r) for r in ranges]
        
        self.publishers_scan[agent_id].publish(msg)

    def cmd_vel_callback(self, msg, agent_id):
        if agent_id in self.clients:
            addr = self.clients[agent_id]
            cmd = {
                "lx": msg.linear.x,
                "az": msg.angular.z
            }
            try:
                packet = json.dumps(cmd).encode('utf-8')
                self.sock.sendto(packet, addr)
            except Exception as e:
                self.get_logger().error(f"Send Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UDPBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
