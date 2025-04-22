import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.msg import SetParametersResult
import math 

class PIDController:
    def __init__(self, p, i, d, reference):
        # Initialiser PID-parameterne og referansen
        self.p = p
        self.i = i
        self.d = d
        self.reference = reference
        self.previous_error = 0  # Holder styr på tidligere feil
        self.integral = 0  # Akkumulert feil (integral)

    def update(self, measured_value, dt):
        # Beregn avvik
        error = self.reference - measured_value

        # Proportional del
        proportional = self.p * error

        # Integral del
        self.integral += error * dt
        integral = self.i * self.integral

        # Derivative del
        derivative = self.d * (error - self.previous_error) / dt if dt > 0 else 0.0
        
        # Oppdater forrige feil
        self.previous_error = error
        
        # Returner samlet PID-kontrollverdi (sum av de tre komponentene)
        return proportional + integral + derivative

class PIDControllerNode(Node):
    def __init__(self):
        # Initialiser ROS2-noden med navnet 'qube_pid_controller'
        super().__init__('qube_pid_controller')

        # Deklarer PID-parametere og referansen som kan settes via ROS2-parametere
        self.declare_parameter('p', 10.0)
        self.declare_parameter('i', 3.0)
        self.declare_parameter('d', 0.3)
        self.declare_parameter('reference', 0.0)

        # Hent parameterne fra ROS2
        self.p = self.get_parameter('p').value
        self.i = self.get_parameter('i').value
        self.d = self.get_parameter('d').value
        self.reference = self.get_parameter('reference').value

        # Initialiser PID-kontrolleren med de hentede parameterne
        self.pid_controller = PIDController(p=self.p, i=self.i, d=self.d, reference=self.reference)

        # Opprett en publisher for å sende hastighetskommandoer til /velocity_controller/commands
        self.velocity_publisher = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        # Abonner på /joint_states for å motta posisjonsdata fra robotens ledd
        self.angle_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Initialiser målte vinkel og siste tid
        self.measured_angle = 0.0
        self.last_time = self.get_clock().now()

        # Timer for å oppdatere PID-kontrolleren hvert 10. millisekund (100 Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Legg til en callback for å håndtere endring av parametere
        self.add_on_set_parameters_callback(self.parameter_callback)

    def joint_state_callback(self, msg):
        # Når nye posisjonsdata mottas, oppdater målte vinkel
        if len(msg.position) > 0:
            self.measured_angle = msg.position[0] 
            self.get_logger().info(f'Qube position: {self.measured_angle:.3f}')

    def timer_callback(self):
        # Denne funksjonen kjøres periodisk for å oppdatere PID-kontrolleren
        current_time = self.get_clock().now()

        # Beregn tidsdifferansen (dt) mellom siste oppdatering og nåværende tid
        dt = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9  # Konverter ns til sekunder
        
        if dt > 0:  # Sørg for at dt er positivt
            # Beregn kontrollkommandoen basert på den målte vinkelen og PID-kontrolleren
            velocity_command = self.pid_controller.update(self.measured_angle, dt)
            self.publish_velocity(velocity_command)

        # Oppdater siste tid for neste beregning
        self.last_time = current_time

    def publish_velocity(self, velocity):
        # Send hastighetskommandoen til /velocity_controller/commands
        msg = Float64MultiArray()
        msg.data = [velocity]  
        self.velocity_publisher.publish(msg)
        self.get_logger().info(f'Published velocity command: {velocity:.3f}')
        
    def parameter_callback(self, params):
        # Denne callbacken håndterer endringer i PID-parametrene
        for param in params:
            # Hvis parameteren er 'p', oppdater PID-kontrolleren
            if param.name == 'p' and param.value >= 0.0:
                self.pid_controller.p = param.value
                self.get_logger().info(f'P updated: {self.pid_controller.p}')
            # Hvis parameteren er 'i', oppdater PID-kontrolleren
            elif param.name == 'i' and param.value >= 0.0:
                self.pid_controller.i = param.value
                self.get_logger().info(f'I updated: {self.pid_controller.i}')
            # Hvis parameteren er 'd', oppdater PID-kontrolleren
            elif param.name == 'd' and param.value >= 0.0:
                self.pid_controller.d = param.value
                self.get_logger().info(f'D updated: {self.pid_controller.d}')
            # Hvis parameteren er 'reference', oppdater PID-kontrolleren (begrens til -π til +π)
            elif param.name == 'reference' and -math.pi <= param.value <= math.pi:
                self.pid_controller.reference = param.value
                self.get_logger().info(f'Reference updated: {self.pid_controller.reference}')

        return SetParametersResult(successful=True)

def main(args=None):
    # Initialiser ROS2-systemet og start PID-kontroller-noden
    rclpy.init(args=args)
    pid_controller_node = PIDControllerNode()
    rclpy.spin(pid_controller_node)
    pid_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # Kjør hovedfunksjonen når scriptet kjøres direkte
    main()

