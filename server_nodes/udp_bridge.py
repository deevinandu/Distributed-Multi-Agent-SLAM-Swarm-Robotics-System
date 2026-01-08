import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import socket
import struct
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
        
        # Quasar-Lite Struct Format
        # < = little endian
        # 4s = magic (4 bytes)
        # B = agent_id (1 byte)
        # f = x (4)
        # f = y (4)
        # f = yaw (4)
        # H = scan_count (2)
        # 181f = ranges (724)
        self.packet_fmt = '<4sBfffH181f'
        self.packet_size = struct.calcsize(self.packet_fmt)
        
        # Command Format: CMD1, linear_x, angular_z
        self.cmd_fmt = '<4sff'

        # Check socket every 0.01s (100Hz)
        self.timer = self.create_timer(0.01, self.udp_callback)
        self.get_logger().info(f"Quasar-Lite Bridge Started. Packet Size: {self.packet_size} bytes")

    def udp_callback(self):
        try:
            ready = select.select([self.sock], [], [], 0.0)
            if ready[0]:
                data, addr = self.sock.recvfrom(65535)
                self.process_packet(data, addr)
        except Exception as e:
            self.get_logger().error(f"UDP Error: {e}")

    def process_packet(self, data, addr):
        if len(data) != self.packet_size:
            self.get_logger().warn(f"Invalid packet size: {len(data)} != {self.packet_size}")
            return

        try:
            # Unpack
            unpacked = struct.unpack(self.packet_fmt, data)
            
            magic = unpacked[0]
            if magic != b'QSRL':
                self.get_logger().warn(f"Invalid Magic: {magic}")
                return

            agent_id = unpacked[1]
            odom_x = unpacked[2]
            odom_y = unpacked[3]
            odom_yaw = unpacked[4]
            # scan_count = unpacked[5] # Unused, assumed 181
            ranges = unpacked[6:] # Tuple of 181 floats

            # Register Client
            if agent_id not in self.clients:
                self.clients[agent_id] = addr
                self.create_agent_interfaces(agent_id)
                self.get_logger().info(f"New Agent Connected: {agent_id} at {addr}")
            
            self.clients[agent_id] = addr

            # Publish
            self.publish_odom(agent_id, odom_x, odom_y, odom_yaw)
            self.publish_scan(agent_id, ranges)

        except Exception as e:
            self.get_logger().error(f"Packet Processing Error: {e}")

    def create_agent_interfaces(self, agent_id):
        aid_str = f"agent_{agent_id}" # e.g. agent_1
        self.publishers_odom[agent_id] = self.create_publisher(
            Odometry, 
            f'/{aid_str}/odom', 
            10
        )
        self.publishers_scan[agent_id] = self.create_publisher(
            LaserScan, 
            f'/{aid_str}/scan', 
            10
        )
        
        self.subscribers[agent_id] = self.create_subscription(
            Twist,
            f'/{aid_str}/cmd_vel',
            lambda m, aid=agent_id: self.cmd_vel_callback(m, aid),
            10
        )

    def publish_odom(self, agent_id, x, y, yaw):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.publishers_odom[agent_id].publish(msg)

    def publish_scan(self, agent_id, ranges):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_link"
        
        # 181 points, -90 to +90
        msg.angle_min = -1.57
        msg.angle_max = 1.57
        msg.angle_increment = 3.14 / 180.0
        msg.range_min = 0.05
        msg.range_max = 2.0
        msg.scan_time = 2.0
        
        msg.ranges = list(ranges) # Convert tuple to list
        
        self.publishers_scan[agent_id].publish(msg)

    def cmd_vel_callback(self, msg, agent_id):
        if agent_id in self.clients:
            addr = self.clients[agent_id]
            try:
                # Pack Command: CMD1 (4s), lx (f), az (f)
                packet = struct.pack(self.cmd_fmt, b'CMD1', msg.linear.x, msg.angular.z)
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
