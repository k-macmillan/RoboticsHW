
#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String

rclpy.init(args=None)

node = rclpy.create_node('publisher')
pub1 = node.create_publisher(String, 'plotpts')
msg = String()

while True:
    message = input("> ")
    if message == 'exit':
        break
    msgarr = message.split(',')
    ch = int(msgarr[1])
    msg.data = msgarr[0]
    if ch == 1:
        pub1.publish(msg)
    if ch == 2:
        pub2.publish(msg)


node.destroy_node()
rclpy.shutdown()
