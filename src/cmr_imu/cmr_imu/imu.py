import rclpy
from rclpy.node import Node
from cmr_msgs.msg import IMUSensorData
import time
import datetime
import platform
import threading
import cmr_imu.device_model as deviceModel
from cmr_imu.jy901s_dataProcessor import JY901SDataProcessor
from cmr_imu.protocol_485_resolver import Protocol485Resolver
from queue import Queue
import math

data_queue = Queue()

#data_list = [temp, accX, accY, accZ, gyroX, gyroY, gyroZ, angleX, angleY, angleZ, magX, magY, magZ]


class IMUPublisher(Node):
    def __init__(self):
        super().__init__('number_publisher')
        self.publisher_ = self.create_publisher(IMUSensorData, '/imu', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_msg)

    def publish_msg(self):
        if not data_queue.empty():
            data_list = data_queue.get()
            msg = IMUSensorData()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.temp = data_list[0]
            # Adjust acceleration from g's to m/s^2
            msg.accx = data_list[1] * 9.81
            msg.accy = data_list[2] * 9.81
            msg.accz = data_list[3] * 9.81
            # Adjust angular velocity from deg/s to rad/s
            msg.gyrox = math.radians(data_list[4])
            msg.gyroy = math.radians(data_list[5])
            msg.gyroz = math.radians(data_list[6])
            msg.anglex = data_list[7]
            msg.angley = data_list[8]
            msg.anglez = data_list[9]
            msg.magx = data_list[10]
            msg.magy = data_list[11]
            msg.magz = data_list[12]
            #msg.data = self.number
            self.publisher_.publish(msg)


_writeF = None                    #写文件  Write file
_IsWriteF = False                 #写文件标识    Write file identification
def readConfig(device):
    """
    读取配置信息示例    Example of reading configuration information
    :param device: 设备模型 Device model
    :return:
    """
    tVals = device.readReg(0x02,3)  #读取数据内容、回传速率、通讯速率   Read data content, return rate, communication rate
    if (len(tVals)>0):
        print("返回结果：" + str(tVals))
    else:
        print("无返回")
    tVals = device.readReg(0x23,2)  #读取安装方向、算法  Read the installation direction and algorithm
    if (len(tVals)>0):
        print("返回结果：" + str(tVals))
    else:
        print("无返回")

def setConfig(device):
    """
    设置配置信息示例    Example setting configuration information
    :param device: 设备模型 Device model
    :return:
    """
    device.unlock()                # 解锁 unlock
    time.sleep(0.1)                # 休眠100毫秒    Sleep 100ms
    device.writeReg(0x03, 6)       # 设置回传速率为10HZ    Set the transmission back rate to 10HZ
    time.sleep(0.1)                # 休眠100毫秒    Sleep 100ms
    device.writeReg(0x23, 0)       # 设置安装方向:水平、垂直   Set the installation direction: horizontal and vertical
    time.sleep(0.1)                # 休眠100毫秒    Sleep 100ms
    device.writeReg(0x24, 0)       # 设置安装方向:九轴、六轴   Set the installation direction: nine axis, six axis
    time.sleep(0.1)                # 休眠100毫秒    Sleep 100ms
    device.save()                  # 保存 Save

def AccelerationCalibration(device):
    """
    加计校准    Acceleration calibration
    :param device: 设备模型 Device model
    :return:
    """
    device.AccelerationCalibration()                 # Acceleration calibration
    print("加计校准结束")

def FiledCalibration(device):
    """
    磁场校准    Magnetic field calibration
    :param device: 设备模型 Device model
    :return:
    """
    device.BeginFiledCalibration()                   # 开始磁场校准   Starting field calibration
    if input("请分别绕XYZ轴慢速转动一圈，三轴转圈完成后，结束校准（Y/N)？").lower()=="y":
        device.EndFiledCalibration()                 # 结束磁场校准   End field calibration
        print("结束磁场校准")

def startRecord():
    """
    开始记录数据  Start recording data
    :return:
    """
    global _IsWriteF
    _IsWriteF = True                                                                        #标记写入标识
    Tempstr = "Chiptime"
    Tempstr +=  "\tax(g)\tay(g)\taz(g)"
    Tempstr += "\twx(deg/s)\twy(deg/s)\twz(deg/s)"
    Tempstr += "\tAngleX(deg)\tAngleY(deg)\tAngleZ(deg)"
    Tempstr += "\tT(°)"
    Tempstr += "\tmagx\tmagy\tmagz"
    Tempstr += "\r\n"
    print("开始记录数据")

def endRecord():
    """
    结束记录数据  End record data
    :return:
    """
    global _IsWriteF
    _IsWriteF = False             

def onUpdate(deviceModel):
    """
    数据更新事件  Data update event
    :param deviceModel: 设备模型    Device model
    :return:
    """
    # print("芯片时间:" + str(deviceModel.getDeviceData("Chiptime"))
    #      , " 温度:" + str(deviceModel.getDeviceData("temperature"))
    #      , " 加速度：" + str(deviceModel.getDeviceData("accX")) +","+  str(deviceModel.getDeviceData("accY")) +","+ str(deviceModel.getDeviceData("accZ"))
    #      ,  " 角速度:" + str(deviceModel.getDeviceData("gyroX")) +","+ str(deviceModel.getDeviceData("gyroY")) +","+ str(deviceModel.getDeviceData("gyroZ"))
    #      , " 角度:" + str(deviceModel.getDeviceData("angleX")) +","+ str(deviceModel.getDeviceData("angleY")) +","+ str(deviceModel.getDeviceData("angleZ"))
    #     , " 磁场:" + str(deviceModel.getDeviceData("magX")) +","+ str(deviceModel.getDeviceData("magY"))+","+ str(deviceModel.getDeviceData("magZ"))
    #       )
    #setting up msg data to send
    data = [deviceModel.getDeviceData("temperature"), 
                 deviceModel.getDeviceData("accX"), deviceModel.getDeviceData("accY"), deviceModel.getDeviceData("accZ"), 
                 deviceModel.getDeviceData("gyroX"), deviceModel.getDeviceData("gyroY"), deviceModel.getDeviceData("gyroZ"), 
                 deviceModel.getDeviceData("angleX"), deviceModel.getDeviceData("angleY"), deviceModel.getDeviceData("angleZ"), 
                 deviceModel.getDeviceData("magX"), deviceModel.getDeviceData("magY"), deviceModel.getDeviceData("magZ")]
    data_queue.put(data)
    # print(data_list)

def LoopReadThead(device):
    """
    循环读取数据  Cyclic read data
    :param device:
    :return:
    """
    while(True):                            #循环读取数据 Cyclic read data
        device.readReg(0x30, 41)            #读取 数据  Read data

def main(args = None):
    # Initialize Publisher Node
    rclpy.init(args=args)
    number_publisher = IMUPublisher()

    device = deviceModel.DeviceModel(
        "我的JY901",
        Protocol485Resolver(),
        JY901SDataProcessor(),
        "51_0"
    )
    device.ADDR = 0x50  # 设置传感器ID Setting the Sensor ID
    if (platform.system().lower() == 'linux'):
        device.serialConfig.portName = "/dev/ttyUSB0"  # 设置串口 Set serial port
    else:
        device.serialConfig.portName = "COM82"  # 设置串口 Set serial port
    device.serialConfig.baud = 9600  # 设置波特率 Set baud rate
    device.openDevice()  # 打开串口 Open serial port
    readConfig(device)  # 读取配置信息 Read configuration information
    device.dataProcessor.onVarChanged.append(onUpdate)  # 数据更新事件 Data update event

    startRecord()  # 开始记录数据 Start recording data
    t = threading.Thread(target=LoopReadThead, args=(device,))  # 开启一个线程读取数据 Start a thread to read data
    t.start()

    # Spin the ROS node in a separate thread to prevent blocking
    ros_thread = threading.Thread(target=rclpy.spin, args=(number_publisher,))
    ros_thread.start()
    
    #device.closeDevice()
    #endRecord()  # 结束记录数据 End record data
    # number_publisher.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
   main()
    