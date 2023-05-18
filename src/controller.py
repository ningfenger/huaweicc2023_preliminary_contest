# coding=utf-8
from robot import *
from workstand import *
import numpy as np
import copy
import math
import logging
from tools import *

# 环境常量
MATCH_FRAME = 3 * 60 * 50  # 总帧数
DISMAP = None  # 初始化时更新，记录任意两个工作台间的测算距离/帧数

ITEMS_NEED = [[] for _ in range(8)]  # 记录收购每个商品的工作台编号

# 控制参数
DIS = [5, 2, 1]
VELO = [5, 2, 1]
# 人工势场常熟
ETA = 600  # 调整斥力大小的常数
GAMMA = 10  # 调整吸引力大小的常数
RADIUS = 6  # 定义斥力半径范围
# 测试
DEBUG = False
time_record = []
robot_start = [0] * 4
robot_dis = [0] * 4
robot_theta = [0] * 4
robot_arrive = [False] * 4


class Controller:
    # 控制参数
    MOVE_SPEED = 1 / 4.15 * 50  # 估算移动时间 4.1 4.2 相同
    MAX_WAIT = 3.14 * 50  # 最大等待时间 3.15
    SELL_WEIGHT = 1.45  # 优先卖给格子被部分占用的 1.43 1.45 相同
    SELL_DEBUFF = 0.8  # 非 7 卖给89的惩罚
    CONSERVATIVE = 1+1/MOVE_SPEED*4  # 保守程度 最后时刻要不要操作 比4.3和3.8好
    BUY_WEIGHT = [1]*4+[1]*3+[1]  # 购买优先级，优先购买高级商品 比0.9和1.1好
    # BUY_WEIGHT = [1.5, 1, 1, 1, 1, 1, 1, 1]  # 购买优先级，优先购买高级商品

    def __init__(self, robots: RobotGroup, workstands: Map):
        self._robots = robots
        self._workstands = workstands
        self._dis_robot2robot = None
        self._dis_robot2cell = None
        self._dis_robot2workstand = None
        self._dis_workstand2workstand = None
        self._dis_cell2workstand = None
        self._dis_robot2workstand2cell = None
        self._delta_x_r2r = None
        self._delta_y_r2r = None
        self._delta_x_w2w = None
        self._delta_y_w2w = None
        self._delta_x_r2w = None
        self._delta_y_r2w = None
        self._delta_x_r2c = None
        self._delta_y_r2c = None
        self._delta_x_c2w = None
        self._delta_y_c2w = None
        self._cooldown_robot2workstand = np.zeros((4, self._workstands.count))
        self._cooldown_workstand2cell = np.zeros(
            (self._workstands.count, self._workstands.count_cell))
        self._time_robot2workstand2cell = np.zeros(
            (4, self._workstands.count, self._workstands.count_cell))
        self._product_workstand_unlock = np.array(
            [True] * workstands.count, dtype=np.bool)
        self._receive_cell_unlock = np.array(
            [True] * workstands.count_cell, dtype=np.bool)
        self._robot_unlock = np.array([True] * 4, dtype=np.bool)
        self._profit_estimation = np.zeros(
            (4, self._workstands.count, self._workstands.count_cell))
        self._profit_rate_estimation = np.zeros(
            (4, self._workstands.count, self._workstands.count_cell))

        self._index_tran_r = [-1] * 4
        self._index_tran_w = [-1] * self._workstands.count
        self._index_tran_c = [-1] * self._workstands.count_cell

        if DEBUG:
            logging.basicConfig(filename='debug.log', level=logging.DEBUG)

    def init_ITEMS_NEED(self):
        workstands = self._workstands
        for idx in range(len(workstands)):
            typeID = int(workstands.get_workstand_status(idx)[0])
            for itemID in WORKSTAND_IN[typeID]:
                ITEMS_NEED[itemID].append(idx)

    def set_control_parameters(self, move_speed: float, max_wait: int, sell_weight: float, sell_debuff: float):
        self.MOVE_SPEED = move_speed  # 估算移动时间
        self.MAX_WAIT = max_wait  # 最大等待时间
        self.SELL_WEIGHT = sell_weight  # 优先卖给格子被部分占用的
        self.SELL_DEBUFF = sell_debuff  # 将456卖给9的惩罚因子

    def cal_dis_robot2robot(self):
        # 计算所有机器人两两之间的距离 向量化 每来一帧调用一次
        # 距离表存在类变量中
        # 通过get_dis_robot2robot(self, idx_robot, idx_workstand)调用
        loc_robots1 = self._robots.get_loc(-1)
        loc_robots2 = self._robots.get_loc(-1)
        self._delta_x_r2r, self._delta_y_r2r, self._dis_robot2robot = get_dx_dy_d(
            loc_robots1, loc_robots2)

    def cal_dis_robot2workstand(self):
        # 计算所有机器人到所有工作站的距离 向量化 每来一帧调用一次
        # 距离表存在类变量中
        # 通过get_dis_robot2workstand(self, idx_robot, idx_workstand)调用
        loc_robots = self._robots.get_loc(-1)
        loc_workstands = self._workstands.get_loc(-1)
        self._delta_x_r2w, self._delta_y_r2w, self._dis_robot2workstand = get_dx_dy_d(
            loc_robots, loc_workstands)

    def cal_dis_workstand2workstand(self):
        # 计算所有工作站两两之间的距离 所有格子到工作站的距离 向量化 只需要在初始化调用一次
        # 距离表存在类变量中
        # 通过get_dis_workstand2workstand(self, idx_workstand1, idx_workstand2)调用
        loc_workstands1 = self._workstands.get_loc(-1)
        loc_workstands2 = self._workstands.get_loc(-1)
        self._delta_x_w2w = loc_workstands1[:, 0].reshape(
            -1, 1) - loc_workstands2[:, 0].reshape(1, -1)
        self._delta_y_w2w = loc_workstands1[:, 1].reshape(
            -1, 1) - loc_workstands2[:, 1].reshape(1, -1)

        self._dis_workstand2workstand = np.sqrt(
            np.power(self._delta_x_w2w, 2) + np.power(self._delta_y_w2w, 2))

        loc_workstand = np.array(
            [[self._workstands.product_workstand_dict[key][0], self._workstands.product_workstand_dict[key][1]] for key
             in self._workstands.product_workstand_dict])

        loc_cell = np.array(
            [[self._workstands.receive_cell_dict[key][0], self._workstands.receive_cell_dict[key][1]] for key in
             self._workstands.receive_cell_dict])
        sell_cell = np.array([self._workstands.receive_cell_dict[key][2]
                             for key in self._workstands.receive_cell_dict])

        self._delta_x_c2w, self._delta_y_c2w, self._dis_cell2workstand = get_dx_dy_d(
            loc_cell, loc_workstand)


    def cal_dis_robot2workstand2cell(self):
        loc_robots = self._robots.get_loc(-1)
        loc_cell = np.array(
            [[self._workstands.receive_cell_dict[key][1], self._workstands.receive_cell_dict[key][0]] for key in
             self._workstands.receive_cell_dict])

        self._delta_x_r2c, self._delta_y_r2c, self._dis_robot2cell = get_dx_dy_d(
            loc_robots, loc_cell)

        self._dis_robot2workstand2cell = self._dis_robot2workstand[:, :, np.newaxis] + self._dis_cell2workstand.T[
            np.newaxis, :, :]

    def cal_profit_rate(self):
        self._profit_rate_estimation = self._profit_estimation / \
            self._time_robot2workstand2cell


    def get_dis_robot2robot(self, idx_robot1, idx_robot2):
        # 机器人到工作台的距离
        return copy.deepcopy(self._dis_robot2robot[idx_robot1, idx_robot2])

    def get_dis_robot2workstand(self, idx_robot, idx_workstand):
        # 机器人到工作台的距离
        idx_robot = int(idx_robot)
        idx_workstand = int(idx_workstand)
        return copy.deepcopy(self._dis_robot2workstand[idx_robot, idx_workstand])

    def get_dis_workstand2workstand(self, idx_workstand1, idx_workstand2):
        # 两个工作台间的距离
        return copy.deepcopy(self._dis_workstand2workstand[idx_workstand1, idx_workstand2])

    def get_delta_theta(self, idx_robot):
        # 机器人头指向 和机器人指向工作台 的夹角
        idx_workstand = int(self._robots.get_status(
            feature_target_r, idx_robot))

        dx_r2w = self._delta_x_r2w[idx_robot,
                                   idx_workstand]
        dy_r2w = self._delta_y_r2w[idx_robot,
                                   idx_workstand]

        target_angle = np.arctan2(dy_r2w, dx_r2w)
        now_theta = self._robots.get_status(feature_theta_r, idx_robot)
        delta_theta = target_angle - now_theta
        delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi
        return delta_theta

    def get_delta_theta2target(self, idx_robot, idx_workstand1, idx_workstand2):
        # 机器人指向工作台1 和工作台1指向工作台2的夹角
        dx_r2w1 = self._delta_x_r2w[idx_robot, idx_workstand1]
        dy_r2w1 = self._delta_y_r2w[idx_robot, idx_workstand1]
        # 机器人指向工作台1 的角度
        target_angle1 = np.arctan2(dy_r2w1, dx_r2w1)
        dx_w12w2 = self._delta_x_w2w[idx_workstand1, idx_workstand2]
        dy_w12w2 = self._delta_y_w2w[idx_workstand1,  idx_workstand2]
        # 工作台1指向工作台2 的角度
        target_angle2 = np.arctan2(dy_w12w2, dx_w12w2)
        return target_angle1 - target_angle2



    def get_other_col_info(self, idx_robot, idx_other):
        # 计算机器人之间的距离
        distance_robot = self.get_dis_robot2robot(idx_robot, idx_other)

        # 自己指向其他的方向余弦
        dx_robot = self._delta_x_r2r[idx_robot,
                                     idx_other] / distance_robot
        # 自己指向其他的方向余弦
        dy_robot = self._delta_y_r2r[idx_robot,
                                     idx_other] / distance_robot

        info_robot = self._robots.get_status(-1, idx_robot)

        info_other = self._robots.get_status(-1, idx_other)

        _, _, _, _, _, vx_robot, vy_robot, theta_robot, x_robot, y_robot, _, target_workstand_robot, _, _, _, _, _ = info_robot
        _, _, _, _, _, vx_other, vy_other, theta_other, x_other, y_other, _, target_workstand_other, _, _, _, _, _ = info_other

        # 判断是否路上正向对撞
        col_flag, x_col, y_col, dist_robot, dist_other = will_collide(x_robot, y_robot, vx_robot, vy_robot, x_other, y_other, vx_other, vy_other, 1.5)
        # 判断是否路上侧向撞上其他机器人
        # 判断是否同时到终点僵持
        return col_flag, x_col, y_col, dist_robot, dist_other

    def upper_speed(self, idx_robot, speed):
        t = 0.3
        info_robot = self._robots.get_status(-1, idx_robot).tolist()
        _, _, _, _, _, vx_robot, vy_robot, theta_robot, x_robot, y_robot, _, target_workstand_robot, _, _, _, _, _ = info_robot
        if 0 < x_robot + t * vx_robot < 50 and 0 < y_robot + t * vy_robot < 50:
            return speed
        else:
            return min(speed / 6, 0.3)

    def get_nearest(self, idx_robot):
        # 检测是否僵持
        near_flag = None
        theta_robot = self._robots.get_status(feature_theta_r, idx_robot)
        dircos_robot = np.array([math.cos(theta_robot), math.sin(theta_robot)])
        for idx_other in range(4):
            if not idx_other == idx_robot:
                # 计算机器人之间的距离
                distance_robot = self.get_dis_robot2robot(idx_robot, idx_other)

                # 自己指向其他的方向余弦
                dx_robot = self._delta_x_r2r[idx_robot,
                                             idx_other] / distance_robot
                # 自己指向其他的方向余弦
                dy_robot = self._delta_y_r2r[idx_robot,
                                             idx_other] / distance_robot

                theta_robot2other = np.arctan2(dy_robot, dx_robot)

                d_theta = theta_robot - theta_robot2other
                d_theta = (d_theta + math.pi) % (2 * math.pi) - math.pi
                if distance_robot < 1.3 and idx_robot < idx_other and abs(d_theta) < math.pi / 2:
                    if d_theta > 0:
                        near_flag = 1
                    else:
                        near_flag = -1
        return near_flag



    def move2loc_new(self, idx_robot):
        info_robot = self._robots.get_status(-1, idx_robot).tolist()
        _, _, _, _, _, vx_robot, vy_robot, theta_robot, x_robot, y_robot, _, target_workstand_robot, _, _, _, _, _ = info_robot
        idx_target = int(self._robots.get_status(feature_target_r, idx_robot))
        target_x = self._workstands.get_status(feature_x_w, idx_target)
        target_y = self._workstands.get_status(feature_y_w, idx_target)

        edge_flag = (x_robot < 1 or x_robot > 49) and (y_robot < 1 or y_robot > 49)
        band_width = 0.9
        offset = 8
        # ########################### 就近 ###########################
        # # 右下角前期虚像
        # if target_x > 49 and target_y < 1:
        #     # 角落
        #     if y_robot > 50 - x_robot:
        #         # ↘的上三角区域
        #         if x_robot < 50 - band_width:
        #             # 右边带外
        #             target_y += offset
        #     else:
        #         # ↘的下三角区域
        #         if y_robot > band_width:
        #             # 下边带外
        #             target_x -= offset
        #
        # # 左上角前期虚像
        # if target_x < 1 and target_y > 49:
        #     # 角落
        #     if y_robot > 50 - x_robot:
        #         # ↖的上三角区域
        #         if y_robot < 50 - band_width:
        #             # 上边带外
        #             target_x += offset
        #     else:
        #         # ↖的下三角区域
        #         if x_robot > band_width:
        #             # 左边带外
        #             target_y -= offset
        #
        # # 左下角前期虚像
        # if target_x < 1 and target_y < 1:
        #     # 角落
        #     if y_robot > x_robot:
        #         # ↙的上三角区域
        #         if x_robot > band_width:
        #             # 左边带外
        #             target_y += offset
        #     else:
        #         # ↙的下三角区域
        #         if y_robot > band_width:
        #             # 下边带外
        #             target_x += offset
        #
        # # 右上角前期虚像
        # if target_x > 49 and target_y > 49:
        #     # 角落
        #     if y_robot > x_robot:
        #         # ↗的上三角区域
        #         if y_robot < 50 - band_width:
        #             # 上边带外
        #             target_x -= offset
        #     else:
        #         # ↗的下三角区域
        #         if x_robot < 50 - band_width:
        #             # 右边带外
        #             target_y -= offset
        # ########################### 就近 ###########################
        ########################### 强制指定进场方向 ###########################
        # 右下角前期虚像
        if target_x > 49 and target_y < 1:
            # 角落
            # ↘的上三角区域
            if y_robot > band_width:
                # 下边带外
                target_x -= offset


        # 左上角前期虚像
        if target_x < 1 and target_y > 49:
            # 角落
            # ↖的下三角区域
            if x_robot > band_width:
                # 左边带外
                target_y -= offset

        # 左下角前期虚像
        if target_x < 1 and target_y < 1:
            # 角落
            # ↙的下三角区域
            if y_robot > band_width:
                # 下边带外
                target_x += offset

        # 右上角前期虚像
        if target_x > 49 and target_y > 49:
            # 角落
            # ↗的上三角区域
            if y_robot < 50 - band_width:
                # 上边带外
                target_x -= offset

        ########################### 强制指定进场方向 ###########################


        distance_r2w = self._dis_robot2workstand[idx_robot, idx_target]
        # 前往目标工作台的方向
        target_theta = np.arctan2(target_y - y_robot, target_x - x_robot)
        delta_theta = target_theta - theta_robot
        delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi

        col_flag = False
        for idx_other in range(4):
            if not idx_other == idx_robot:
                col_flag, x_col, y_col, dist_robot, dist_other = self.get_other_col_info(idx_robot, idx_other)
                if col_flag:
                    break
        k_r = 10
        n_r = 1
        k_s = 10
        n_s = 1.5
        d_far = 6
        d_near = 4
        d_daoche = 3

        ang_large = math.pi * 0.9
        ang_small = math.pi * 0.1
        near_flag = self.get_nearest(idx_robot)
        if near_flag is not None:
            # 僵持不下
            # 接触僵持 不避免僵持
            self._robots.forward(idx_robot, -2)
            delta_theta += near_flag * math.pi / 2
            delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi
            self._robots.rotate(idx_robot, delta_theta * k_r)
        elif col_flag and dist_robot > dist_other:
            # raise Exception('1')
            if dist_robot > 3:
                speed = (dist_robot - 3) / 3
            else:
                if self._robots.get_status(feature_workstand_id_r, idx_robot) == -1:
                    speed = 0
                else:
                    speed = -2
                # delta_theta += math.pi / 4
            self._robots.forward(idx_robot, -1)
            self._robots.rotate(idx_robot, delta_theta * k_r)
        elif edge_flag and abs(delta_theta) > ang_small:
            # 原地转（角落容易碰撞）
            self._robots.rotate(
                idx_robot, sign_pow(delta_theta, n_r) * k_r)
            self._robots.forward(idx_robot, 0)
        else:


            if abs(delta_theta) < ang_small:
                # 面对目标方向



                self._robots.rotate(
                    idx_robot, sign_pow(delta_theta, n_r) * k_r)
                self._robots.forward(idx_robot, self.upper_speed(idx_robot, distance_r2w ** n_s * k_s))
            elif abs(delta_theta) < ang_large:
                if distance_r2w > d_far:
                    # 距离远且不面对目标方向

                    # 边开边转（有希望开近的时候面对目标）
                    self._robots.rotate(
                        idx_robot, sign_pow(delta_theta, n_r) * k_r)
                    self._robots.forward(idx_robot, self.upper_speed(idx_robot, distance_r2w ** n_s * k_s))
                elif distance_r2w > d_near:
                    # 距离适中且不面对目标方向

                    # 低速前进地转（防止凑近之后绕圈）
                    self._robots.rotate(
                        idx_robot, sign_pow(delta_theta, n_r) * k_r)
                    self._robots.forward(idx_robot, self.upper_speed(idx_robot, 2))
                else:
                    # 距离近且不面对目标方向

                    # 原地转（防止凑近之后绕圈）
                    self._robots.rotate(
                        idx_robot, sign_pow(delta_theta, n_r) * k_r)
                    self._robots.forward(idx_robot, 0)
            else:
                if distance_r2w > d_far:
                    # 距离远且背对目标方向
                    # 倒车转向
                    self._robots.rotate(
                        idx_robot, sign_pow(delta_theta, n_r) * k_r)
                    self._robots.forward(idx_robot, -2)
                elif distance_r2w > d_near:
                    # 距离适中且背对目标方向
                    # 倒车转向
                    self._robots.rotate(
                        idx_robot, sign_pow(delta_theta, n_r) * k_r)
                    self._robots.forward(idx_robot, -2)
                elif distance_r2w > d_daoche:
                    # 距离较近且背对目标方向
                    # 倒车转向
                    self._robots.rotate(
                        idx_robot, sign_pow(delta_theta, n_r) * k_r)
                    self._robots.forward(idx_robot, -2)
                else:
                    # 倒车距离且背对目标方向 √

                    # 倒车
                    delta_theta += math.pi
                    self._robots.rotate(
                        idx_robot, sign_pow(delta_theta, n_r) * k_r)
                    self._robots.forward(idx_robot, -distance_r2w ** n_s * k_s)


    def calculate_potential_field(self, idx_robot, idx_workstand):
        # 计算位于current_pos处的机器人的势能场
        near_flag = -1
        idx_workstand = int(idx_workstand)
        attractive_field = np.zeros(2)
        repulsive_field = np.zeros(2)
        theta_robot = self._robots.get_status(feature_theta_r, idx_robot)
        dircos_robot = np.array([math.cos(theta_robot), math.sin(theta_robot)])
        for idx_other in range(4):
            if not idx_other == idx_robot:
                # 计算机器人之间的距离
                distance_robot = self.get_dis_robot2robot(idx_robot, idx_other)

                # 自己指向其他的方向余弦
                dx_robot = self._delta_x_r2r[idx_robot,
                                             idx_other] / distance_robot
                # 自己指向其他的方向余弦
                dy_robot = self._delta_y_r2r[idx_robot,
                                             idx_other] / distance_robot
                theta2other = np.arctan2(dy_robot, dx_robot)
                theta_shun = np.arctan2(dx_robot, -dy_robot)  # (-y, x)
                theta_ni = np.arctan2(-dx_robot, dy_robot)  # (y, -x)
                theta_other = self._robots.get_status(
                    feature_theta_r, idx_other)
                theta_robot = self._robots.get_status(
                    feature_theta_r, idx_robot)
                # 各向异性判断是否会产生潜在的碰撞事件

                # 其他人的头朝向方向余弦
                dircos_other = np.array(
                    [math.cos(theta_other), math.sin(theta_other)])

                # 自己的头朝向方向余弦
                dircos_robot = np.array(
                    [math.cos(theta_robot), math.sin(theta_robot)])

                dircos_robot2other = np.array([dx_robot, dy_robot])
                
                # 头朝向方向 与 指向其他机器人方向 的 夹角
                ang_robot = math.acos(
                    np.dot(dircos_robot, np.array([dx_robot, dy_robot])))

                ang_other = math.acos(
                    np.dot(dircos_other, np.array([-dx_robot, -dy_robot])))

                v_robot = math.sqrt(self._robots.get_status(feature_line_velo_x_r, idx_robot)
                                    ** 2 + self._robots.get_status(feature_line_velo_y_r, idx_robot) ** 2)
                v_other = math.sqrt(self._robots.get_status(feature_line_velo_x_r, idx_other)
                                    ** 2 + self._robots.get_status(feature_line_velo_y_r, idx_other) ** 2)
                m_robot = self._robots.get_status(
                    feature_materials_r, idx_robot)
                m_other = self._robots.get_status(
                    feature_materials_r, idx_other)
                if distance_robot < 2 and idx_robot < idx_other and math.acos(np.dot(dircos_robot2other, dircos_robot)) < math.pi / 4:
                    near_flag = idx_other

                # # 如果机器人之间的距离小于一定半径范围，则计算斥力
                # and (ang_robot < math.pi * 0.3 or ang_other < math.pi * 0.3):
                if distance_robot < RADIUS and ang_robot < math.pi / 4:  # and m_robot <= m_other and idx_robot < idx_other:
                    repulsive_force = 0.5 * idx_robot * ETA * \
                        ((1.0 / distance_robot) - (1.0 / RADIUS)) ** 2

                    repulsive_field[0] += repulsive_force * dy_robot
                    repulsive_field[1] -= repulsive_force * dx_robot

        # 计算机器人到目标点的吸引力
        distance_r2w = self.get_dis_robot2workstand(idx_robot, idx_workstand)

        dx_r2w = self._delta_x_r2w[idx_robot,
                                   idx_workstand] / distance_r2w  # 自己指向工作台
        dy_r2w = self._delta_y_r2w[idx_robot,
                                   idx_workstand] / distance_r2w  # 自己指向工作台

        attractive_force = 0.5 * GAMMA * distance_r2w ** 2
        attractive_field[0] = attractive_force * dx_r2w
        attractive_field[1] = attractive_force * dy_r2w

        total_field = repulsive_field + attractive_field
        desired_angle = np.arctan2(total_field[1], total_field[0])
        return desired_angle, distance_r2w, near_flag

    def pre_rotate(self, idx_robot, next_walkstand):
        desired_theta, _, _ = self.calculate_potential_field(
            idx_robot, next_walkstand)
        now_theta = self._robots.get_status(feature_theta_r, idx_robot)
        delta_theta = desired_theta - now_theta
        delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi
        n_r = 1.5
        k_r = 15
        self._robots.rotate(idx_robot, sign_pow(delta_theta, n_r) * k_r)

    def get_time_rate(self, frame_sell: float) -> float:
        # 计算时间损失
        if frame_sell >= 9000:
            return 0.8
        sqrt_num = math.sqrt(1 - (1 - frame_sell / 9000) ** 2)
        return (1 - sqrt_num) * 0.2 + 0.8

    def set_pro_buy(self, idx_robot: int, buy=True):
        # 设置预定
        target_walkstand, _ = self._robots.robots_plan[idx_robot]
        # 预定工作台
        if buy:
            workstand_type = int(
                self._workstands.get_workstand_status(target_walkstand)[0])
            # if workstand_type in [1, 2, 3]:  # 123不锁
            #     return
            self._workstands.set_product_pro(target_walkstand, 1)
        else:
            self._workstands.set_product_pro(target_walkstand, 0)

    def set_pro_sell(self, idx_robot: int, sell=True):
        # 设置预售
        target_walkstand, next_walkstand = self._robots.robots_plan[idx_robot]
        material_pro = int(
            self._workstands.get_material_pro(next_walkstand))
        workstand_types = int(
            self._workstands.get_workstand_status(target_walkstand)[0])
        next_walkstand_type = int(
            self._workstands.get_workstand_status(next_walkstand)[0])
        if next_walkstand_type in [8, 9]:
            return
        if sell:
            self._workstands.set_material_pro(
                next_walkstand, material_pro + (1 << workstand_types))
        else:
            self._workstands.set_material_pro(
                next_walkstand, material_pro - (1 << workstand_types))

    def count_1(self, v):  # 记录每个数中含有多少个1
        num = 0
        while v:
            num += v & 1
            v >>= 1
        return num

    def choise(self, frame_id: int, idx_robot: int) -> bool:
        # 进行一次决策
        max_radio = 0  # 记录最优性价比
        for idx_workstand in range(len(self._workstands)):
            workstand_type, product_time, material, product_status = map(
                int, self._workstands.get_workstand_status(idx_workstand))
            if WORKSTAND_OUT[workstand_type] == None or product_time == -1 and product_status == 0:  # 不生产
                continue
            if int(self._workstands.get_product_pro(idx_workstand)) == 1:  # 被预定了,后序考虑优化
                continue
            frame_wait_buy = product_time if product_status == 0 else 0  # 生产所需时间，如果已有商品则为0
            if frame_wait_buy > self.MAX_WAIT:
                continue
            frame_move_to_buy = self.get_dis_robot2workstand(
                idx_robot, idx_workstand) * self.MOVE_SPEED
            buy_weight = self.BUY_WEIGHT[workstand_type]
            # 需要这个产品的工作台
            for idx_worksand_to_sell in ITEMS_NEED[workstand_type]:
                sell_type, sell_product_time, sell_material, sell_product_status = map(
                    int, self._workstands.get_workstand_status(idx_worksand_to_sell))
                # 这个格子已被预定
                if 1 << workstand_type & int(self._workstands.get_material_pro(idx_worksand_to_sell)):
                    continue
                # frame_wait_sell = 0
                # 格子里有这个原料
                if DEBUG:
                    logging.debug(
                        f"frame_id:{frame_id}, idx_robot:{idx_robot}, idx_worksand_to_sell:{idx_worksand_to_sell}")
                    logging.debug(
                        f"sell_product_time:{sell_product_time}")
                    logging.debug(
                        f"workstand_type:{workstand_type}, sell_material:{sell_material}")
                # 判断是不是8或9 不是8或9 且这个原料格子已经被占用的情况, 生产完了并不一定能继续生产
                # 2023-3-22 优化如果格子已满可以考虑
                frame_wait_sell = 0
                if WORKSTAND_OUT[sell_type] and 1 << workstand_type & sell_material:
                    continue
                    # 阻塞或者材料格没满
                    if sell_product_time in [-1, 0] or sell_material != WORKSTAND_FULL[sell_type]:
                        continue
                    elif sell_product_status == 1:  # 到这里说明材料格和产品格都满了不会再消耗原料格了
                        continue
                    else:
                        frame_wait_sell = sell_product_time
                # frame_move_to_buy, frame_move_to_sell= self.get_time_rww(idx_robot, idx_workstand, idx_worksand_to_sell)
                frame_move_to_sell = self.get_dis_workstand2workstand(
                    idx_workstand, idx_worksand_to_sell) * self.MOVE_SPEED
                frame_buy = max(frame_move_to_buy,
                                frame_wait_buy)  # 购买时间
                frame_sell = max(frame_move_to_sell,
                                 frame_wait_sell - frame_buy)  # 出售时间
                total_frame = frame_buy + frame_sell  # 总时间
                if total_frame*self.CONSERVATIVE + frame_id > MATCH_FRAME:  # 完成这套动作就超时了
                    continue
                time_rate = self.get_time_rate(
                    frame_move_to_sell)  # 时间损耗
                sell_weight = self.SELL_WEIGHT**self.count_1(sell_material) # 已经占用的格子越多优先级越高
                sell_weight = self.SELL_WEIGHT if sell_material else 1 # 已经占用的格子越多优先级越高
                sell_debuff = self.SELL_DEBUFF if sell_type == 9 and workstand_type != 7 else 1
                radio = (
                    ITEMS_SELL[workstand_type] * time_rate - ITEMS_BUY[
                        workstand_type]) / total_frame*sell_weight*buy_weight*sell_debuff
                if radio > max_radio:
                    max_radio = radio
                    self._robots.robots_plan[idx_robot] = [
                        idx_workstand, idx_worksand_to_sell]  # 更新计划
        if max_radio > 0:  # 开始执行计划
            # 设置机器人移动目标
            target_walkstand, next_walkstand = self._robots.robots_plan[idx_robot]
            self._robots.set_status_item(
                feature_target_r, idx_robot, target_walkstand)
            # 预定工作台
            self.set_pro_buy(idx_robot)
            self.set_pro_sell(idx_robot)
            self._robots.set_status_item(
                feature_status_r, idx_robot, RobotGroup.MOVE_TO_BUY_STATUS)
            return True
        return False


    def control(self, frame_id: int):
        self.cal_dis_robot2workstand()
        self.cal_dis_robot2robot()
        self.cal_dis_robot2workstand2cell()
        idx_robot = 0
        sell_out_list = []  # 等待处理预售的机器人列表
        while idx_robot < 4:
            robot_status = int(self._robots.get_status(
                feature_status_r, idx_robot))
            if robot_status == RobotGroup.FREE_STATUS:
                # 【空闲】执行调度策略

                if self.choise(frame_id, idx_robot):
                    # 记录任务分配时的时间 距离 角度相差
                    robot_start[idx_robot] = frame_id
                    robot_dis[idx_robot] = self._dis_robot2workstand[
                        idx_robot, int(self._robots.get_status(feature_target_r, idx_robot))]
                    robot_theta[idx_robot] = self.get_delta_theta(idx_robot)
                    robot_arrive[idx_robot] = False
                    # 记录任务分配时的时间 距离 角度相差
                    continue
            elif robot_status == RobotGroup.MOVE_TO_BUY_STATUS:
                # 【购买途中】

                self.move2loc_new(idx_robot)

                # 判定是否进入交互范围
                if self._robots.get_status(feature_workstand_id_r, idx_robot) == self._robots.get_status(
                        feature_target_r, idx_robot):
                    # 记录任务分配时的时间 距离 角度相差
                    if robot_arrive[idx_robot] == False:
                        delta_time = frame_id - robot_start[idx_robot]
                        robot_arrive[idx_robot] = True
                        time_record.append(
                            [robot_dis[idx_robot], robot_theta[idx_robot], delta_time])
                    # 记录任务分配时的时间 距离 角度相差
                    self._robots.set_status_item(
                        feature_status_r, idx_robot, RobotGroup.WAIT_TO_BUY_STATUS)  # 切换为 【等待购买】
                    continue
            elif robot_status == RobotGroup.WAIT_TO_BUY_STATUS:
                # 【等待购买】
                target_walkstand, next_walkstand = self._robots.robots_plan[idx_robot]
                self.pre_rotate(idx_robot, next_walkstand)
                product_status = int(
                    self._workstands.get_workstand_status(target_walkstand)[3])
                # 如果在等待，提前转向
                if product_status == 1:  # 这里判定是否生产完成可以购买 不是真的1
                    # 可以购买
                    if self._robots.buy(idx_robot):  # 防止购买失败
                        self.set_pro_buy(idx_robot, False)
                        self._robots.set_status_item(
                            feature_target_r, idx_robot, next_walkstand)  # 更新目标到卖出地点
                        self._robots.set_status_item(
                            feature_status_r, idx_robot, RobotGroup.MOVE_TO_SELL_STATUS)  # 切换为 【出售途中】
                        # logging.debug(f"{idx_robot}->way to sell")
                        # 记录任务分配时的时间 距离 角度相差
                        robot_start[idx_robot] = frame_id
                        robot_dis[idx_robot] = self._dis_robot2workstand[
                            idx_robot, int(self._robots.get_status(feature_target_r, idx_robot))]
                        robot_theta[idx_robot] = self.get_delta_theta(
                            idx_robot)
                        robot_arrive[idx_robot] = False
                        # 记录任务分配时的时间 距离 角度相差
                        continue
                    else:
                        self._robots.set_status_item(
                            feature_status_r, idx_robot, RobotGroup.MOVE_TO_BUY_STATUS)  # 购买失败说明位置不对，切换为 【购买途中】
                        continue

            elif robot_status == RobotGroup.MOVE_TO_SELL_STATUS:
                # 【出售途中】
                # 移动
                # 判断距离是否够近

                self.move2loc_new(idx_robot)

                # 判定是否进入交互范围
                if self._robots.get_status(feature_workstand_id_r, idx_robot) == self._robots.get_status(
                        feature_target_r, idx_robot):
                    self._robots.set_status_item(
                        feature_status_r, idx_robot, RobotGroup.WAIT_TO_SELL_STATUS)  # 切换为 【等待出售】
                    # logging.debug(f"{idx_robot}->ready to sell")
                    # 记录任务分配时的时间 距离 角度相差
                    if robot_arrive[idx_robot] == False:
                        delta_time = frame_id - robot_start[idx_robot]
                        robot_arrive[idx_robot] = True
                        time_record.append(
                            [robot_dis[idx_robot], robot_theta[idx_robot], delta_time])
                    # 记录任务分配时的时间 距离 角度相差
                    continue

            elif robot_status == RobotGroup.WAIT_TO_SELL_STATUS:
                # 【等待出售】
                _, target_walkstand = self._robots.robots_plan[idx_robot]
                workstand_type, _, material, _ = map(
                    int, self._workstands.get_workstand_status(target_walkstand))
                material_type = int(self._robots.get_status(
                    feature_materials_r, idx_robot))
                # 如果在等待，提前转向
                # raise Exception(F"{material},{material_type}")
                # 这里判定是否生产完成可以出售 不是真的1
                if not WORKSTAND_OUT[workstand_type] or (material & 1 << material_type) == 0:
                    # 可以购买
                    if self._robots.sell(idx_robot):  # 防止出售失败
                        # 取消预定
                        sell_out_list.append(idx_robot)
                        self._robots.set_status_item(
                            feature_status_r, idx_robot, RobotGroup.FREE_STATUS)  # 切换为空闲
                        # logging.debug(f"{idx_robot}->wait")
                        break  # 防止别人以它为目标 后面可以优化改为最后集中处理预售
                    else:
                        self._robots.set_status_item(
                            feature_status_r, idx_robot, RobotGroup.MOVE_TO_SELL_STATUS)  # 购买失败说明位置不对，切换为 【出售途中】
                        continue
            idx_robot += 1
        for idx_robot in sell_out_list:
            self.set_pro_sell(idx_robot, False)
