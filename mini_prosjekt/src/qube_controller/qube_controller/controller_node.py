import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.msg import SetParametersResult
import math 

class PIDController:
    def __init__(self, p, i, d, reference):
        self.p = p
        self.i = i
        self.d = d
        self.reference = reference
        self.previous_error = 0
        self.integral = 0

    def update(self, measured_value, dt):
        error = self.reference - measured_value
        proportional = self.p * error
        self.integral += error * dt
        integral = self.i * self.integral
        derivative = self.d * (error - self.previous_error) / dt if dt > 0 else 0.0
        
        self.previous_error = error
        return proportional + integral + derivative

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('qube_pid_controller')
        
        # Deklarer PID-parametere + referanse
        self.declare_parameter('p', 10.0)
        self.declare_parameter('i', 3.0)
        self.declare_parameter('d', 0.3)
        self.declare_parameter('reference', 0.0)

        self.p = self.get_parameter('p').value
        self.i = self.get_parameter('i').value
        self.d = self.get_parameter('d').value
        self.reference = self.get_parameter('reference').value

        self.pid_controller = PIDController(p=self.p, i=self.i, d=self.d, reference=self.reference)

        # Publisher til velocity controller
        self.velocity_publisher = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        # Abonnerer på /joint_states
        self.angle_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.measured_angle = 0.0
        self.last_time = self.get_clock().now()

        # PID-oppdatering hver 10ms (100Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def joint_state_callback(self, msg):
        if len(msg.position) > 0:
            self.measured_angle = msg.position[0]  # Hent første ledds posisjon
            self.get_logger().info(f'Qube position: {self.measured_angle:.3f}')

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9  # Konverter ns til s
        
        if dt > 0:
            velocity_command = self.pid_controller.update(self.measured_angle, dt)
            self.publish_velocity(velocity_command)

        self.last_time = current_time

    def publish_velocity(self, velocity):
        msg = Float64MultiArray()
        msg.data = [velocity]  
        self.velocity_publisher.publish(msg)
        self.get_logger().info(f'Published velocity command: {velocity:.3f}')
        
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'p' and param.value >= 0.0:
                self.pid_controller.p = param.value
                self.get_logger().info(f'P updated: {self.pid_controller.p}')
            elif param.name == 'i' and param.value >= 0.0:
                self.pid_controller.i = param.value
                self.get_logger().info(f'I updated: {self.pid_controller.i}')
            elif param.name == 'd' and param.value >= 0.0:
                self.pid_controller.d = param.value
                self.get_logger().info(f'D updated: {self.pid_controller.d}')
            elif param.name == 'reference' and -math.pi <= param.value <= math.pi:
                self.pid_controller.reference = param.value
                self.get_logger().info(f'Reference updated: {self.pid_controller.reference}')

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    pid_controller_node = PIDControllerNode()
    rclpy.spin(pid_controller_node)
    pid_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

