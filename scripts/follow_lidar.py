#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# --- CẤU HÌNH ---
TARGET_DISTANCE = 0.5  # Khoảng cách muốn giữ (mét)
KP = 0.5               # Độ nhạy
MAX_SPEED = 0.3        # Tốc độ tối đa (m/s)

class LidarFollower:
    def __init__(self):
        rospy.init_node('lidar_follower_node')
        
        # Gửi lệnh xuống topic /cmd_vel để ESP32 nhận
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Lắng nghe Lidar từ topic /scan
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        
        self.cmd = Twist()
        rospy.loginfo("DA KHOI DONG: Che do bam duoi Lidar!")

    def scan_cb(self, msg):
        # Lấy dữ liệu Lidar
        ranges = msg.ranges
        
        # Lấy góc giữa (phía trước mặt robot)
        # Giả sử lidar quét 360 độ, phần tử giữa mảng là phía trước
        mid = len(ranges) // 2
        
        # Lấy trung bình cộng 10 tia quét ở giữa để cho chính xác
        valid_ranges = []
        for i in range(mid - 5, mid + 5):
            dist = ranges[i]
            # Chỉ lấy giá trị hợp lệ (0.1m đến 3m)
            if 0.1 < dist < 3.0:
                valid_ranges.append(dist)
        
        if len(valid_ranges) == 0:
            return # Không thấy gì thì không làm gì

        avg_dist = sum(valid_ranges) / len(valid_ranges)

        # Tính sai số (Khoảng cách thực tế - Khoảng cách mong muốn)
        error = avg_dist - TARGET_DISTANCE
        
        # Điều khiển P (Proportional)
        speed = error * KP

        # Giới hạn tốc độ
        if speed > MAX_SPEED: speed = MAX_SPEED
        if speed < -MAX_SPEED: speed = -MAX_SPEED
        
        # Deadzone: Nếu sai số nhỏ quá (< 5cm) thì đứng yên
        if abs(error) < 0.05:
            speed = 0.0

        # Gửi lệnh
        self.cmd.linear.x = speed
        self.cmd.angular.z = 0.0
        self.pub.publish(self.cmd)

if __name__ == '__main__':
    try:
        LidarFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
