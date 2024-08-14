import sys
sys.path.append('./uservo.py')
import Robot
import serial
import threading
from data_pub.uservo import UartServoManager

class MasterRobot():

    def __init__(self, robot_name):
        self.robot_name = robot_name
        if self.robot_name == "master_left":
            self.SERVO_PORTS_ARM = [
                "/dev/ttyCH9344USB0",
                "/dev/ttyCH9344USB1",
                "/dev/ttyCH9344USB2",
                "/dev/ttyCH9344USB3",
                "/dev/ttyCH9344USB4",
                "/dev/ttyCH9344USB5",
            ]
            self.SERVO_PORTS_GRIPPER = ["/dev/ttyCH9344USB6", "/dev/ttyCH9344USB7"] 
            # self.SERVO_PORTS_GRIPPER = ["/dev/ttyCH9344USB6"] 
            self.SERVO_IDS_ARM = [0, 1, 2, 3, 4, 5]
            self.SERVO_IDS_GRIPPER = [6, 17]
            # self.SERVO_IDS_GRIPPER = [6]
            self.MASTER_HOME_POS_ARM = [6, 4, -6, -4, 91, -2]
            self.MASTER_OPEN = -21.3
            self.MASTER_CLOSE = -62
            self.SLAVE_OPEN = -27
            self.SLAVE_CLOSE = 42

        elif self.robot_name == "master_right":
            self.SERVO_PORTS_ARM = [
                "/dev/ttyCH9344USB8",
                "/dev/ttyCH9344USB9",
                "/dev/ttyCH9344USB10",
                "/dev/ttyCH9344USB11",
                "/dev/ttyCH9344USB12",
                "/dev/ttyCH9344USB13",
            ]
            self.SERVO_PORTS_GRIPPER = ["/dev/ttyCH9344USB14", "/dev/ttyCH9344USB15"] 
            # self.SERVO_PORTS_GRIPPER = ["/dev/ttyCH9344USB14"] 
            self.SERVO_IDS_ARM = [10, 11, 12, 13, 14, 15]
            self.SERVO_IDS_GRIPPER = [16, 7]
            # self.SERVO_IDS_GRIPPER = [16]
            self.MASTER_HOME_POS_ARM = [-46, 93, -82, -4, -96, 14]   
            self.MASTER_OPEN = 5
            self.MASTER_CLOSE = 59
            self.SLAVE_OPEN = -32
            self.SLAVE_CLOSE = 41
          
     
        self.MASTER_HOME_POS_GRIPPER = [self.MASTER_OPEN, self.SLAVE_OPEN]
        self.SLAVE_HOME_POS_ARM = [45.0, -90.0, -90.0, -0.0, 90.0, 0.0]  # 从机械臂初始角度 
        self.SERVO_BAUDRATE = 115200 
        self.uart_managers_arm = []
        self.uart_managers_gripper = []
    
    def initialize_servos(self):
     
        for port in self.SERVO_PORTS_ARM:
            try:
                print(f"初始化左机械臂串口 {port}")
                uart = serial.Serial(port=port, 
                                     baudrate=self.SERVO_BAUDRATE,
                                     parity=serial.PARITY_NONE, 
                                     stopbits=1, 
                                     bytesize=8, 
                                     timeout=0)
                self.uart_managers_arm.append(UartServoManager(uart))
                print(f"左机械臂串口 {port} 初始化成功")
            except serial.SerialException as e:
                print(f"左机械臂串口 {port} 初始化失败: {e}")

        for port in self.SERVO_PORTS_GRIPPER:
            try:
                print(f"初始化串口 {port}")
                uart = serial.Serial(
                    port=port,
                    baudrate=self.SERVO_BAUDRATE,
                    parity=serial.PARITY_NONE,
                    stopbits=1,
                    bytesize=8,
                    timeout=0,
                )
                self.uart_managers_gripper.append(UartServoManager(uart))
                print(f"串口 {port} 初始化成功")
            except serial.SerialException as e:
                print(f"串口 {port} 初始化失败: {e}")

    def set_initial_positions(self):
        # 设置机械臂舵机初始角度ba
        print("设置机械臂舵机初始角度")
        for i, uservo in enumerate(self.uart_managers_arm):
            uservo.set_servo_angle(
                self.SERVO_IDS_ARM[i],
                self.MASTER_HOME_POS_ARM[i],
                velocity=50.0,
                t_acc=500,
                t_dec=500,
            )
            uservo.wait()  # 等待舵机静止
            print(f"舵机 {self.SERVO_IDS_ARM[i]} 设为初始角度 {self.MASTER_HOME_POS_ARM[i]}")

        # 设置夹爪舵机初始角度
        print("设置夹爪舵机初始角度")
        for i, uservo in enumerate(self.uart_managers_gripper):
            uservo.set_servo_angle(
                self.SERVO_IDS_GRIPPER[i], self.MASTER_HOME_POS_GRIPPER[i], interval=0
            )  # 设置舵机角度 极速模式
            uservo.wait()  # 等待舵机静止
            print(f"舵机 {self.SERVO_IDS_GRIPPER[i]} 设为初始角度 {self.MASTER_HOME_POS_GRIPPER[i]}")
        print("设置夹爪舵机初始角度")
        for i, uservo in enumerate(self.uart_managers_gripper):
            uservo.set_servo_angle(
                self.SERVO_IDS_GRIPPER[i], self.MASTER_HOME_POS_GRIPPER[i], interval=0
            )  # 设置舵机角度 极速模式
            uservo.wait()  # 等待舵机静止
            print(f"舵机 {self.SERVO_IDS_GRIPPER[i]} 设为初始角度 {self.MASTER_HOME_POS_GRIPPER[i]}")

        # 初始化夹爪舵机
        for port in self.SERVO_PORTS_GRIPPER:
            try:
                print(f"初始化串口 {port}")
                uart = serial.Serial(
                    port=port,
                    baudrate=self.SERVO_BAUDRATE,
                    parity=serial.PARITY_NONE,
                    stopbits=1,
                    bytesize=8,
                    timeout=0,
                )
                self.uart_managers_gripper.append(UartServoManager(uart))
                print(f"串口 {port} 初始化成功")
            except serial.SerialException as e:
                print(f"串口 {port} 初始化失败: {e}")
    
    def set_damping_mode(self):
        # 设置读取角度的舵机为阻尼模式
        print("设置夹爪读取角度的舵机为阻尼模式")
        self.uart_managers_gripper[0].set_damping(self.SERVO_IDS_GRIPPER[0], 10)
        print(f"舵机 {self.SERVO_IDS_GRIPPER[0]} 阻尼模式设置完成")

        # 设置所有舵机为阻尼模式
        print("设置机械臂舵机为阻尼模式")
        for i, uservo in enumerate(self.uart_managers_arm):
            uservo.set_damping(self.SERVO_IDS_ARM[i], 50)
            print(f"舵机 {self.SERVO_IDS_ARM[i]} 阻尼模式设置完成")

    

    def get_robot_data(self):
        # Shared data between threads
        joint_pos = [0, 0, 0, 0, 0, 0]
        grip_percentage = [0] 
        def read_servo_angle(index, uservo, servo_id):
            angle = uservo.query_servo_angle(servo_id)
            if index == 2:
                joint_pos[index] = self.SLAVE_HOME_POS_ARM[index] + (angle - self.MASTER_HOME_POS_ARM[index])
            else:
                joint_pos[index] = self.SLAVE_HOME_POS_ARM[index] - (angle - self.MASTER_HOME_POS_ARM[index])

        def read_gripper_angle():
            angle_gripper = self.uart_managers_gripper[0].query_servo_angle(self.SERVO_IDS_GRIPPER[0])
            grip_percentage[0] = (self.MASTER_OPEN - angle_gripper) / (self.MASTER_OPEN - self.MASTER_CLOSE)

        threads = []
        for i, uservo in enumerate(self.uart_managers_arm):
            t = threading.Thread(
                target=read_servo_angle,
                args=(i, uservo, self.SERVO_IDS_ARM[i])
            )
            threads.append(t)
            t.start()

        gripper_thread = threading.Thread(target=read_gripper_angle)
        threads.append(gripper_thread)
        gripper_thread.start()

        for t in threads:
            t.join()

        slave_angle = grip_percentage[0] * (self.SLAVE_CLOSE - self.SLAVE_OPEN) + self.SLAVE_OPEN
        self.uart_managers_gripper[1].set_servo_angle(
            self.SERVO_IDS_GRIPPER[1], slave_angle, interval=0
        )

        return joint_pos, grip_percentage[0]


class PuppetRobot():
    def __init__(self, robot_ip):
        self.robot_ip = robot_ip
        self.robot = Robot.RPC(self.robot_ip)
    

    def initialize_robot(self, joint_pos_o):
        # 机器人初始化
        print("初始化从机械臂")
        ret = self.robot.MoveJ(joint_pos_o, tool=0, user=0, vel=10)
        print("从机械臂就位", ret)
        # error, joint_pos = self.robot.GetActualJointPosDegree()
        # if error != 0:
        #     print(f"获取机器人当前关节位置失败，错误码: {error}")
        #     sys.exit(1)
        # print("机器人当前关节位置", joint_pos)
    
    def robot_control(self, joint_pos):
        ret = self.robot.ServoJ(joint_pos, cmdT=0.008)

     


