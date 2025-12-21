#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import serial
import time

# --- CẤU HÌNH ---
SERIAL_PORT = '/dev/ttyUSB1' 
BAUD_RATE = 115200

class SimpleSerialSender:
    def __init__(self):
        rospy.init_node('serial_string_sender')
        
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            rospy.loginfo(f"Da ket noi voi ESP32 (Cong thuc V/A) qua {SERIAL_PORT}")
            time.sleep(2) 
        except Exception as e:
            rospy.logerr(f"Khong mo duoc cong Serial: {e}")
            return

        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.callback)
        
    def callback(self, msg):
        # vx: ly (tiến lùi)
        # w:  rx (xoay)
        vx = msg.linear.x  
        w  = msg.angular.z 

        # --- BIẾN ĐỔI THEO CÔNG THỨC CỦA BẠN ---
        # m1 = -ly - rx -> fl
        # m2 = +ly - rx -> fr
        # m3 =  ly - rx -> rl
        # m4 = -ly - rx -> rr (Lưu ý: m1 và m4 trong công thức bạn giống hệt nhau)
        
        fl = -vx - w
        fr =  vx - w
        rl =  vx - w
        rr = -vx - w

        # --- TẠO CHUỖI VĂN BẢN ---
        # Gửi dạng chuỗi để ESP32 sscanf ra được
        data_string = f"{fl:.2f},{fr:.2f},{rl:.2f},{rr:.2f}\n"

        try:
            self.ser.write(data_string.encode('utf-8'))
            rospy.loginfo(f"Gui chuoi: {data_string.strip()}")
        except Exception as e:
            rospy.logwarn(f"Loi gui serial: {e}")

if __name__ == '__main__':
    try:
        sender = SimpleSerialSender()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass





# import rospy
# from geometry_msgs.msg import Twist
# import serial # Thư viện giao tiếp Serial thuần
# import time

# # --- CẤU HÌNH ---
# SERIAL_PORT = '/dev/ttyUSB1' # Cổng của ESP32 (Kiểm tra lại nhé)
# BAUD_RATE = 115200

# # Thông số xe
# L = 0.20
# W = 0.16
# K = L + W

# class SimpleSerialSender:
#     def __init__(self):
#         rospy.init_node('serial_string_sender')
        
#         # 1. Mở cổng Serial (Kết nối trực tiếp với ESP32)
#         try:
#             self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
#             rospy.loginfo(f"Da ket noi voi ESP32 qua {SERIAL_PORT}")
#             time.sleep(2) # Chờ ESP32 khởi động lại
#         except Exception as e:
#             rospy.logerr(f"Khong mo duoc cong Serial: {e}")
#             return

#         self.sub = rospy.Subscriber('/cmd_vel', Twist, self.callback)
        
#     def callback(self, msg):
#         vx = msg.linear.x
#         vy = msg.linear.y
#         w  = msg.angular.z

#         # Tính toán (m/s)
#         fl = vx - vy - (K * w)
#         fr = vx + vy + (K * w)
#         rl = vx + vy - (K * w)
#         rr = vx - vy + (K * w)

#         # --- TẠO CHUỖI VĂN BẢN ---
#         # Format: "fl,fr,rl,rr" (Ví dụ: "0.50,0.50,0.50,0.50")
#         data_string = f"{fl:.2f},{fr:.2f},{rl:.2f},{rr:.2f}\n"

#         # Gửi xuống ESP32
#         self.ser.write(data_string.encode('utf-8'))
        
#         # In ra màn hình máy tính để bạn kiểm tra
#         print(f"Gui chuoi: {data_string.strip()}")

# if __name__ == '__main__':
#     try:
#         SimpleSerialSender()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass