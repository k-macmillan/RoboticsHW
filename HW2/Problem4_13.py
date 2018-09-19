import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D

pt0 = (0.0, 0.0)
pt1 = (0.0, 15.0)
pt2 = (5.0, 20.0)

theta0 = 0.0
theta1 = 0.0
theta2 = 0.0

dist_threshold = 1.0

def dist(pt, xy):
    pass

def thetadiff(goal, theta):
    return True

def callback(msg):
    print('x,y:   {},{}'.format(msg.x, msg.y))
    print('theta: {}'.format(msg.theta))
    if dist(pt0, (msg.x, msg.y)) < dist_threshold and thetadiff(theta0, msg.theta):
        # Rotate
        print('rotate')
    else:
        # Have obtained correct angle, now move forward
        new_msg = Float32()
        new_msg.data = 2.0
        publeft.publish(new_msg)
        pubright.publish(new_msg)



if __name__ == '__main__':


    node_name = 'circle'
    rclpy.init()
    node = Node(node_name)

    print('Spinning: {}'.format(node_name))
    publeft = node.create_publisher(Float32, '/wheel_left')
    pubright = node.create_publisher(Float32, '/wheel_right')
    pos = node.create_subscription(Pose2D, '/GPS', callback)
    

    msg = Float32()

    msg.data = 2.0
    publeft.publish(msg)
    pubright.publish(msg)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutting down...')
        msg.data = 0.0
        publeft.publish(msg)
        pubright.publish(msg)
        node.destroy_node()
        rclpy.shutdown()