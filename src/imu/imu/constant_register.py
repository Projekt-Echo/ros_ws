# constant_register.py
# -*- coding: utf-8 -*-
"""
定义JY61P传感器的寄存器地址常量
"""

# 保存/重启/恢复出厂
SAVE = 0x00
# 校准模式
CALSW = 0x01
# 输出内容
RSW = 0x02
# 输出速率
RRATE = 0x03
# 串口波特率
BAUD = 0x04
# 加速度X零偏
AXOFFSET = 0x05
# 加速度Y零偏
AYOFFSET = 0x06
# 加速度Z零偏
AZOFFSET = 0x07
# 角速度X零偏
GXOFFSET = 0x08
# 角速度Y零偏
GYOFFSET = 0x09
# 角速度Z零偏
GZOFFSET = 0x0A
# 磁场X零偏
HXOFFSET = 0x0B
# 磁场Y零偏
HYOFFSET = 0x0C
# 磁场Z零偏
HZOFFSET = 0x0D
# D0引脚模式
D0MODE = 0x0E
# D1引脚模式
D1MODE = 0x0F
# D2引脚模式
D2MODE = 0x10
# D3引脚模式
D3MODE = 0x11
# 设备地址
IICADDR = 0x1A
# 关闭LED灯
LEDOFF = 0x1B
# 磁场X校准范围
MAGRANGX = 0x1C
# 磁场Y校准范围
MAGRANGY = 0x1D
# 磁场Z校准范围
MAGRANGZ = 0x1E
# 带宽
BANDWIDTH = 0x1F
# 陀螺仪量程
GYRORANGE = 0x20
# 加速度量程
ACCRANGE = 0x21
# 休眠
SLEEP = 0x22
# 安装方向
ORIENT = 0x23
# 算法
AXIS6 = 0x24
# 动态滤波
FILTK = 0x25
# GPS波特率
GPSBAUD = 0x26
# 读取寄存器
READADDR = 0x27
# 加速度滤波
ACCFILT = 0x2A
# 指令启动
POWONSEND = 0x2D
# 版本号
VERSION = 0x2E
# 年月
YYMM = 0x30
# 日时
DDHH = 0x31
# 分秒
MMSS = 0x32
# 毫秒
MS = 0x33
# 加速度X
AX = 0x34
# 加速度Y
AY = 0x35
# 加速度Z
AZ = 0x36
# 角速度X
GX = 0x37
# 角速度Y
GY = 0x38
# 角速度Z
GZ = 0x39
# 磁场X
HX = 0x3A
# 磁场Y
HY = 0x3B
# 磁场Z
HZ = 0x3C
# 横滚角
Roll = 0x3D
# 俯仰角
Pitch = 0x3E
# 航向角
Yaw = 0x3F
# 温度
TEMP = 0x40
# D0引脚状态
D0Status = 0x41
# D1引脚状态
D1Status = 0x42
# D2引脚状态
D2Status = 0x43
# D3引脚状态
D3Status = 0x44
# 气压低16位
PressureL = 0x45
# 气压高16位
PressureH = 0x46
# 高度低16位
HeightL = 0x47
# 高度高16位
HeightH = 0x48
# 经度低16位
LonL = 0x49
# 经度高16位
LonH = 0x4A
# 纬度低16位
LatL = 0x4B
# 纬度高16位
LatH = 0x4C
# GPS海拔
GPSHeight = 0x4D
# GPS航向角
GPSYAW = 0x4E
# GPS地速低16位
GPSVL = 0x4F
# GPS地速高16位
GPSVH = 0x50
# 四元数0
q0 = 0x51
# 四元数1
q1 = 0x52
# 四元数2
q2 = 0x53
# 四元数3
q3 = 0x54
# 卫星数
SVNUM = 0x55
# 位置精度
PDOP = 0x56
# 水平精度
HDOP = 0x57
# 垂直精度
VDOP = 0x58
# 报警信号延时
DELAYT = 0x59
# X轴角度报警最小值
XMIN = 0x5A
# X轴角度报警最大值
XMAX = 0x5B
# 供电电压
BATVAL = 0x5C
# 报警引脚映射
ALARMPIN = 0x5D
# Y轴角度报警最小值
YMIN = 0x5E
# Y轴角度报警最大值
YMAX = 0x5F
# 陀螺仪静止阈值
GYROCALITHR = 0x61
# 角度报警电平
ALARMLEVEL = 0x62
# 陀螺仪自动校准时间
GYROCALTIME = 0x63
# 报警连续触发时间
TRIGTIME = 0x68
# 解锁
KEY = 0x69
# 陀螺仪变化值
WERROR = 0x6A
# GPS时区
TIMEZONE = 0x6B
# 角速度连续静止时间
WZTIME = 0x6E
# 角速度积分阈值
WZSTATIC = 0x6F
# 485数据应答延时
MODDELAY = 0x74
# 横滚角零位参考值
XREFROLL = 0x79
# 俯仰角零位参考值
YREFPITCH = 0x7A
# 设备编号1-2
NUMBERID1 = 0x7F
# 设备编号3-4
NUMBERID2 = 0x80
# 设备编号5-6
NUMBERID3 = 0x81
# 设备编号7-8
NUMBERID4 = 0x82
# 设备编号9-10
NUMBERID5 = 0x83
# 设备编号11-12
NUMBERID6 = 0x84
