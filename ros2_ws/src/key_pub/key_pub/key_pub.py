import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int32

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

class key_pub(Node):
    def __init__(self):
        super().__init__("Key_pub")
        settings = saveTerminalSettings()
        self.publisher_ = self.create_publisher(Int32, "/key", 10)
        #key_timeout = rclpy.get_param("~key_timeout", 0.5)
        run = True
        while(run == True):
            keycode = getKey(settings)
            print(keycode)
            key = Int32()
            publish = True

            if keycode == "q":
                run = False

            if keycode == "a":
                key.data = 0
                self.publisher_.publish(key)
            if keycode == "w":
                key.data = 1
                self.publisher_.publish(key)
            if keycode == "d":
                key.data = 2
                self.publisher_.publish(key)
            if keycode == "s":
                key.data = 3
                self.publisher_.publish(key)


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [])
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def main(args=None):
    rclpy.init(args=args)

    Key_pub = key_pub()



    rclpy.spin(Key_pub)

    Key_pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()