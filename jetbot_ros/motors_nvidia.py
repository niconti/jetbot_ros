import rclpy
from jetbot_ros.motors import MotorController
from Adafruit_MotorHAT import Adafruit_MotorHAT


class NvidiaMotorController(MotorController):
    """
    Motor controller node that supports the original NVIDIA JetBot.
    @see motors.py for the base class to implement different controllers.
    """

    def __init__(self, driver: Adafruit_MotorHAT):
        """
        """
        super().__init__()
        self.MOTOR_LEFT = 1     # Left motor ID
        self.MOTOR_RIGHT = 4    # Right motor ID
        self._driver = driver
        self._motors = {
            self.MOTOR_LEFT : self._driver.getMotor(self.MOTOR_LEFT),
            self.MOTOR_RIGHT : self._driver.getMotor(self.MOTOR_RIGHT)
        }


    def _set_pwm(self, motor_id: int, value: float, trim: float):
        """
        Set motor pwm
        """
        # expand value to PWM range [0, MAX_PWM]
        pwm = (trim + abs(value)) * self.max_pwm
        pwm = max(0, pwm)
        pwm = min(pwm, self.max_pwm)
        pwm = int(pwm)
        self._motors[motor_id].setSpeed(pwm)

        # set the motor direction
        cmd = Adafruit_MotorHAT.RELEASE
        if value > 0:
            cmd = Adafruit_MotorHAT.FORWARD
        if value < 0:
            cmd = Adafruit_MotorHAT.BACKWARD    
        self._motors[motor_id].run(cmd)


    def set_speed(self, left: float, right: float):
        """
        Sets the motor speeds between [-1.0, 1.0]
        """
        self._set_pwm(self.MOTOR_LEFT, left, self.left_trim)
        self._set_pwm(self.MOTOR_RIGHT, right, self.right_trim)


def main(args=None):
    rclpy.init(args=args)
    
    # Init
    try:
        driver = Adafruit_MotorHAT(i2c_bus=7)
    except OSError as err:
        name = f"{__name__}"
        rclpy.logging.get_logger(name).fatal("{}, init fail.".format(err))
        exit(1)
    node = NvidiaMotorController(driver)

    # Spin
    try:
        rclpy.spin(node)
    except KeyboardInterrupt as err:
        node.get_logger().debug("user asked to shutdown")

    # Shutdown
    node.destroy_node()

    
if __name__ == '__main__':
    main()
    
	

