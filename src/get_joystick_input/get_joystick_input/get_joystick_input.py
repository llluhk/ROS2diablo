import rclpy
from rclpy.node import Node
from custom_msgs.msg import Panel
import serial

# Quellen: https://roboticsbackend.com/ros2-create-custom-message/ ##

class CurrentInput(Node):
    def __init__(self):
        super().__init__('current_input')
        self.panel_pub = self.create_publisher(Panel, 'carrierbot/Panel', 10)
        #创建发布器用于发布名为carrierbot/Panel topic.
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        #声明一个参数来设置串口连接：/dev/ttyUSB1 是ROS2默认USB串口设备路径
        # 登陆DIABLO，连接USB后通过： dmesg | grep tty  获取设备路径
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        #这行代码获取刚才声明的参数，并将其转换成字符串类型
        self.baud_rate = 115200
        #设置串口通信的传输速度
        try:
            self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.serial_port}: {e}")
            self.serial_connection = None
        #通过pyserial库创建了一个串口连接对象，创建完成后就可以进行数据的读取和写入
        self.timer = self.create_timer(0.05, self.on_timer)
        
        
    def on_timer(self):  
        #定时器回调函数，20hz的频率发布  
        if self.serial_connection.in_waiting > 0:
            #self.serial_connection.in_waiting 表示当前串口输入缓冲区中等待被读取的字节数。
            line = self.serial_connection.readline().decode('utf-8').strip()
            #解析数据
            if line:
                #self.get_logger().info(f'line recieved: {line}')
                values = line.split("\t")
                # publish read serial data
                panel_msg = Panel()
                #Panel是costom_msg定义的消息类型
                if abs(int(values[1])) > 10:
                    #如果 values=["123", "-45", "1", "0", "1", "0", "1", "0", "1"], 则vaules[0] =123
                    panel_msg.joystick.x = int(values[1])    # x and y swapped because coordinate frame of robot is turned by 90° compared to the frame of the potentiometer
                if abs(int(values[0])) > 10:
                    #values[1] = -45
                    panel_msg.joystick.y = -int(values[0])      # *(-1) because y-Axis points to the left
                panel_msg.joystick.button = int(values[2])
                panel_msg.mainbuttons.upbutton = int(values[3])
                panel_msg.mainbuttons.rightbutton = int(values[4])
                panel_msg.mainbuttons.downbutton = int(values[5])
                panel_msg.mainbuttons.leftbutton = int(values[6])
                panel_msg.sidebuttons.ebutton = int(values[7])
                panel_msg.sidebuttons.fbutton = int(values[8])
                self.panel_pub.publish(panel_msg)
                
def main(args=None):
    rclpy.init(args=args)
    node = CurrentInput()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()