import math
# ROS 2
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult


class MotorController(Node):
    """
    Abstract motor controller base node for supporting different JetBots.
    Can be extended to support any diff drive by overriding set_speed(),
    or any node that subscribes to the /jetbot/cmd_vel Twist message.
    """
    def __init__(self):
        super().__init__('motors', namespace='jetbot')
        self._has_changed = True
        self._vx = 0.0
        self._wz = 0.0

        # Parameters
        self.declare_parameter('left_trim', 0.0)
        self.declare_parameter('right_trim', 0.0)
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('max_rpm', 200)              # https://www.adafruit.com/product/3777
        self.declare_parameter('wheel_separation', 0.1016)  # 4 inches
        self.declare_parameter('wheel_diameter', 0.060325)  # 2 3/8 inches

        self.left_trim = self.get_parameter('left_trim').value
        self.right_trim = self.get_parameter('right_trim').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value        
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Subscriptions
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.command_velocity_cb, 10)


    @property
    def vx(self):
        return self._vx

    @vx.setter
    def vx(self, value: float):
        if self.vx != value:
            self._has_changed = True
        self._vx = value

    @property
    def wz(self):
        return self._wz

    @wz.setter
    def wz(self, value: float):
        if self.wz != value:
            self._has_changed = True
        self._wz = value


    def has_changed(self) -> bool:
        value = self._has_changed
        self._has_changed = False
        return value


    def destroy_node(self):
        self.get_logger().info(f"shutting down, stopping robot...")
        self.stop()


    def parameters_callback(self, params):
        """
        Callback called when a parameters update is requested.
        """
        for param in params:
            if param.name == 'left_trim':
                self.left_trim = param.value
                continue
            if param.name == 'right_trim':
                self.right_trim = param.value
                continue
            if param.name == 'max_pwm':
                self.max_pwm = param.value
                continue
            if param.name == 'wheel_separation':
                self.wheel_separation = param.value
                continue
            raise ValueError(f'unknown parameter {param.name}')

        return SetParametersResult(successful=True)
        

    def set_speed(self, left, right):
        """
        Sets the motor speeds between [-1.0, 1.0]
        Override this function for other motor controller setups.
        Should take into account left_trim, right_trim, and max_pwm.
        """
        raise NotImplementedError('MotorController subclasses should implement set_speed()')


    def stop(self):
        """
        Stop the robot.
        """
        self.set_speed(0, 0)


    def command_velocity_cb(self, msg: Twist):
        """
        Callback called when a command velocity message arrives.
        """
        self.vx = msg.linear.x
        self.wz = msg.angular.z
        
        if not self.has_changed():
            return

        v_left  = self.vx - self.wz * self.wheel_separation / 2.0
        v_right = self.vx + self.wz * self.wheel_separation / 2.0

        w_left  = v_left  / (2.0 * math.pi * (self.wheel_diameter / 2.0))
        w_right = v_right / (2.0 * math.pi * (self.wheel_diameter / 2.0))

        max_rps = self.max_rpm / 60.0
        max_speed = max_rps * 2.0 * math.pi * (self.wheel_diameter / 2.0)

        v_left = min(v_left, max_speed)
        v_left = max(-max_speed, v_left)
        v_left /= max_speed

        v_right = min(v_right, max_speed)
        v_right = max(-max_speed, v_right)
        v_right /= max_speed

        text = f"vx={self.vx:6.03f} wz={self.wz:6.03f} -> v_left={v_left:6.03f} v_right={v_right:6.03f}  (max_speed={max_speed:5.03f} m/s)"
        self.get_logger().debug(text)
        self.set_speed(v_left, v_right)
