import rclpy
from nav_controller.cuadradoBasic import *

def pathMapeo():
    #return [[-3.00, -4.00, -4.00, -2.00, 0, 2.00, 2.00, 0], [0, 0, -2.00, -2.00, -2.00, -2.00, 0, 0]]
    #return [[1.00, 2.00, 2.00, 0],[2.00, 2.00, 0, 0]]
    return [[0],[0]]
def main(args=None) -> None:
    rclpy.init(args=args)
    robot = Trajectory('nodeTrajectoryBasic',pathMapeo=pathMapeo())
        
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()