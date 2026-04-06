import rclpy
from jetbot_ros.motors import MotorController

import board
from adafruit_motorkit import MotorKit


class AdafruitMotorController(MotorController):
    """
    Motor controller node that supports the Adafruit DC & Stepper Motor FeatherWing.
    @see motors.py for the base class to implement different controllers.
    """
    
    def __init__(self, kit: MotorKit):
        """
        """
        super().__init__()
        self._kit = kit


    def set_speed(self, left, right):
        """
        Sets the motor speeds between [-1.0, 1.0]
        """
        self.motor1.throttle = left
        self.motor4.throttle = right
 

def main(args=None):
    rclpy.init(args=args)
    
    # Init
    try:
        kit = MotorKit(i2c=board.I2C())
    except OSError as err:
        name = f"{__name__}"
        rclpy.logging.get_logger(name).fatal("{}, init fail.".format(err))
        exit(1)
    node = AdafruitMotorController(kit)

    # Spin    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt as err:
        node.get_logger().debug("user asked to shutdown")

    # Shutdown
    node.destroy_node()
    
    
if __name__ == '__main__':
    main()
