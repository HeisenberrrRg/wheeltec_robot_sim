#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
import math

# ================= 🔧 调参实验室 =================
DIFF_THRESHOLD = 0.15    # 动态判定阈值 (米)
R_STD = 0.05             # 测量噪声 (越小越信雷达，越大越平滑)
Q_STD = 0.15             # 过程噪声 (越大越灵敏，越小越稳重)
PREDICT_DURATION = 2.0   # 预测未来几秒的轨迹
OBJECT_RADIUS = 0.25     # 【新增】几何补偿半径 (假设圆柱体/人的半径)
# ===============================================

class SimpleKalmanFilter:
    def __init__(self):
        # 状态向量: [x, y, vx, vy]
        self.x = np.zeros(4) 
        self.P = np.eye(4) * 1.0
        
        # 观测矩阵 H (只观测位置 x, y)
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        
        # 噪声矩阵
        self.R = np.eye(2) * (R_STD ** 2)
        self.Q = np.eye(4) * (Q_STD ** 2)
        
        self.initialized = False

    def predict(self, dt):
        # 状态转移矩阵 F (匀速模型)
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        
        # 动态过程噪声 (时间越长不确定性越大)
        q_pos = 0.5 * (dt**2) * Q_STD
        q_vel = dt * Q_STD
        self.Q = np.diag([q_pos, q_pos, q_vel, q_vel]) ** 2

        # 预测步骤
        self.x = np.dot(F, self.x)
        self.P = np.dot(np.dot(F, self.P), F.T) + self.Q
        return self.x

    def update(self, z):
        if not self.initialized:
            self.x[0], self.x[1] = z[0], z[1]
            self.initialized = True
            return

        # 卡尔曼增益 K
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        try:
            K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        except np.linalg.LinAlgError:
            return 
        
        # 更新步骤
        y = z - np.dot(self.H, self.x)
        self.x = self.x + np.dot(K, y)
        I = np.eye(4)
        self.P = np.dot((I - np.dot(K, self.H)), self.P)

class PedestrianTracker(Node):
    def __init__(self):
        super().__init__('pedestrian_tracker')
        
        # 订阅雷达 (Best Effort QoS)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data) 
        
        # 发布可视化 Marker
        self.marker_pub = self.create_publisher(MarkerArray, '/tracker_markers', 10)
        
        self.kf = SimpleKalmanFilter()
        self.last_ranges = None
        self.last_time = None 
        
        self.get_logger().info("【终极修正版】追踪节点已启动！包含几何中心补偿功能...")

    def scan_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        current_ranges = np.array(msg.ranges)
        
        # 初始化
        if self.last_ranges is None:
            self.last_ranges = current_ranges
            self.last_time = current_time
            return
        
        # 计算 dt
        dt = current_time - self.last_time
        if dt <= 0.001: return 
        
        # 1. 提取动态点
        dynamic_pts = self.extract_dynamic_points(msg, current_ranges, self.last_ranges)
        self.last_ranges = current_ranges 
        self.last_time = current_time 
        
        if len(dynamic_pts) < 5: return 

        # 2. 计算表面中心 (Surface Mean)
        surface_x = np.mean(dynamic_pts[:, 0])
        surface_y = np.mean(dynamic_pts[:, 1])

        # === 【核心Trick】几何中心补偿 ===
        # 原理：雷达扫到的是表面，真实中心在连线延长线上
        angle = math.atan2(surface_y, surface_x)
        center_x = surface_x + OBJECT_RADIUS * math.cos(angle)
        center_y = surface_y + OBJECT_RADIUS * math.sin(angle)
        z_measurement = np.array([center_x, center_y])
        # ===============================

        # 3. 高斯拟合 (用于画青色椭圆，使用原始表面点即可)
        mean_gaussian, cov_gaussian = self.fit_gaussian(dynamic_pts)

        # 4. 卡尔曼滤波 (使用补偿后的中心进行更新)
        self.kf.predict(dt)
        self.kf.update(z_measurement)
        
        est_x, est_y = self.kf.x[0], self.kf.x[1]
        est_vx, est_vy = self.kf.x[2], self.kf.x[3]
        
        # 5. 发布全套可视化
        # 注意：这里传给可视化的 mean/cov 依然用高斯的，但位置球用 KF 估计的
        self.publish_all_markers(est_x, est_y, est_vx, est_vy, mean_gaussian, cov_gaussian, msg.header.frame_id)

    def extract_dynamic_points(self, scan_msg, ranges, last_ranges):
        if len(ranges) != len(last_ranges): return np.empty((0, 2))
        
        # 处理 inf/nan
        r1 = np.nan_to_num(ranges, posinf=0.0, neginf=0.0)
        r2 = np.nan_to_num(last_ranges, posinf=0.0, neginf=0.0)
        
        # 差分提取
        diff = np.abs(r1 - r2)
        valid_mask = (ranges > scan_msg.range_min) & (ranges < scan_msg.range_max)
        dynamic_mask = (diff > DIFF_THRESHOLD) & valid_mask
        
        indices = np.where(dynamic_mask)[0]
        if len(indices) == 0: return np.empty((0, 2))

        # 极坐标转直角坐标
        angles = scan_msg.angle_min + indices * scan_msg.angle_increment
        r = ranges[indices]
        x = r * np.cos(angles)
        y = r * np.sin(angles)
        return np.column_stack((x, y))

    def fit_gaussian(self, points):
        if len(points) < 5: return None, None
        mean = np.mean(points, axis=0)
        cov = np.cov(points, rowvar=False)
        return mean, cov

    def generate_ellipse_points(self, mean, cov, n_std=2.0, num_points=30):
        """生成青色椭圆的点集"""
        if mean is None or cov is None: return []
        vals, vecs = np.linalg.eigh(cov)
        order = vals.argsort()[::-1]
        vals = vals[order]
        vecs = vecs[:, order]
        theta = np.degrees(np.arctan2(*vecs[:, 0][::-1]))
        width, height = 2 * n_std * np.sqrt(vals)
        angle_rad = np.radians(theta)
        
        points = []
        for i in range(num_points + 1):
            t = 2 * np.pi * i / num_points
            ell_x = (width/2) * np.cos(t)
            ell_y = (height/2) * np.sin(t)
            rot_x = ell_x * np.cos(angle_rad) - ell_y * np.sin(angle_rad) + mean[0]
            rot_y = ell_x * np.sin(angle_rad) + ell_y * np.cos(angle_rad) + mean[1]
            p = Point()
            p.x, p.y, p.z = rot_x, rot_y, 0.5
            points.append(p)
        return points

    def generate_predicted_path(self, x, y, vx, vy, duration=2.0, step=0.2):
        """生成黄色预测轨迹"""
        path_points = []
        t = 0.0
        while t <= duration:
            p = Point()
            p.x = x + vx * t
            p.y = y + vy * t
            p.z = 0.5
            path_points.append(p)
            t += step
        return path_points

    def publish_all_markers(self, x, y, vx, vy, mean, cov, frame_id):
        marker_array = MarkerArray()
        
        # 基础设置 (时间戳归零，防抖动)
        base_marker = Marker()
        base_marker.header.frame_id = frame_id
        base_marker.header.stamp.sec = 0
        base_marker.header.stamp.nanosec = 0
        base_marker.action = Marker.ADD
        base_marker.lifetime.nanosec = 200000000 # 0.2s 自动消失

        # 1. 绿色球 (修正后的中心位置)
        m_pos = Marker()
        m_pos.header = base_marker.header
        m_pos.ns, m_pos.id, m_pos.type = "pos", 0, Marker.SPHERE
        m_pos.pose.position.x, m_pos.pose.position.y, m_pos.pose.position.z = x, y, 0.5
        m_pos.scale.x = m_pos.scale.y = m_pos.scale.z = 0.3
        m_pos.color.a, m_pos.color.g = 1.0, 1.0
        marker_array.markers.append(m_pos)

        # 2. 紫色箭头 (速度矢量)
        m_vel = Marker()
        m_vel.header = base_marker.header
        m_vel.ns, m_vel.id, m_vel.type = "vel", 1, Marker.ARROW
        p1 = Point(); p1.x, p1.y, p1.z = x, y, 0.5
        p2 = Point(); p2.x, p2.y, p2.z = x + vx, y + vy, 0.5
        m_vel.points = [p1, p2]
        m_vel.scale.x, m_vel.scale.y, m_vel.scale.z = 0.1, 0.2, 0.1
        m_vel.color.a, m_vel.color.r, m_vel.color.b = 1.0, 0.8, 0.8
        marker_array.markers.append(m_vel)

        # 3. 黄色线条 (预测轨迹)
        m_path = Marker()
        m_path.header = base_marker.header
        m_path.ns, m_path.id, m_path.type = "predict", 2, Marker.LINE_STRIP
        m_path.points = self.generate_predicted_path(x, y, vx, vy, PREDICT_DURATION)
        m_path.scale.x = 0.05
        m_path.color.a, m_path.color.r, m_path.color.g = 1.0, 1.0, 1.0
        m_path.color.b = 0.0 # 黄色
        marker_array.markers.append(m_path)

        # 4. 青色椭圆 (高斯分布形状)
        # 只有在点数足够计算出高斯分布时才画
        if mean is not None:
            m_cov = Marker()
            m_cov.header = base_marker.header
            m_cov.ns, m_cov.id, m_cov.type = "shape", 3, Marker.LINE_STRIP
            m_cov.points = self.generate_ellipse_points(mean, cov)
            m_cov.scale.x = 0.05
            m_cov.color.a, m_cov.color.g, m_cov.color.b = 1.0, 1.0, 1.0 # 青色
            marker_array.markers.append(m_cov)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(PedestrianTracker())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()