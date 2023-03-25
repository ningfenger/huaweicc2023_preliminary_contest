# coding=utf-8
from robot import *
from workstand import *
import numpy as np
import copy
import math
import logging

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


def sign_pow(num_in, n):
    if num_in < 0:
        return -abs(num_in) ** n
    else:
        return abs(num_in) ** n


def compute_time_to_arrive(dis, theta):
    return 7.81833903 * abs(dis) + abs(theta) * 19.57800632


def count_ones(n):
    # 初始化计数器
    n = int(n)
    count = 0
    # 循环计算二进制位中数字1的数量
    while n > 0:
        count += n & 1
        n >>= 1
    # 返回结果
    return count


def get_dx_dy_d(a, b):
    dx = b[:, 0].reshape(1, -1) - a[:, 0].reshape(-1, 1)
    dy = b[:, 1].reshape(1, -1) - a[:, 1].reshape(-1, 1)
    d = np.sqrt(np.power(dx, 2) + np.power(dy, 2))
    return dx, dy, d

def will_collide(x1, y1, vx1, vy1, x2, y2, vx2, vy2, t_max, r = 0.53):
    # 计算机器人之间的初始距离
    dist = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    # 计算相对速度
    rel_vx = vx1 - vx2
    rel_vy = vy1 - vy2
    # 如果机器人的相对速度为零，则它们永远不会相遇
    if rel_vx == 0 and rel_vy == 0:
        return (False, None, None, None, None)
    # 计算参数方程
    a = rel_vx**2 + rel_vy**2
    b = 2 * ((x1 - x2) * rel_vx + (y1 - y2) * rel_vy)
    c = (x1 - x2)**2 + (y1 - y2)**2 - 4 * r**2
    delta = b**2 - 4 * a * c
    # 如果delta小于零，则机器人之间不会相遇
    if delta < 0:
        return (False, None, None, None, None)
    else:
        t1 = (-b + math.sqrt(delta)) / (2 * a)
        t2 = (-b - math.sqrt(delta)) / (2 * a)
        t = min(t1, t2)
        # 如果时间是负数或者超出了预测的时间范围，则机器人之间不会相遇
        if t < 0 or t > t_max:
            return (False, None, None, None, None)
        # 计算碰撞点的位置
        collision_x = (x1 + vx1 * t + x2 + vx2 * t) / 2
        collision_y = (y1 + vy1 * t + y2 + vy2 * t) / 2
        # 计算碰撞点距离各自的长度
        distance1 = math.sqrt((collision_x - x1)**2 + (collision_y - y1)**2) - r
        distance2 = math.sqrt((collision_x - x2)**2 + (collision_y - y2)**2) - r
        return (True, collision_x, collision_y, distance1, distance2)



def will_collide2(x_robot, y_robot, vx_robot, vy_robot, x_other, y_other, vx_other, vy_other, t):
    # 计算机器人和另一个物体之间的相对速度
    vx_rel = vx_robot - vx_other
    vy_rel = vy_robot - vy_other

    # 计算机器人和另一个物体之间的相对位置
    x_rel = x_robot - x_other
    y_rel = y_robot - y_other

    # 如果两个物体在x和y方向上的相对速度和相对位置都相反，则它们会碰撞
    if vx_rel * x_rel + vy_rel * y_rel < 0:
        # 计算碰撞时间
        t_col = -(vx_rel * x_rel + vy_rel * y_rel) / (vx_rel ** 2 + vy_rel ** 2)

        # 计算碰撞点的坐标
        x_col = x_robot + vx_robot * t_col
        y_col = y_robot + vy_robot * t_col

        # 计算碰撞点距离各自的长度
        dist_robot = ((x_col - x_robot) ** 2 + (y_col - y_robot) ** 2) ** 0.5
        dist_other = ((x_col - x_other) ** 2 + (y_col - y_other) ** 2) ** 0.5

        # 判断碰撞时间是否在t时间内
        if t_col < t:
            return True, x_col, y_col, dist_robot, dist_other

    return False, None, None, None, None


class Controller:
    # 控制参数
    MOVE_SPEED = 1 / 4 * 50  # 估算移动时间
    MAX_WAIT = 3 * 50  # 最大等待时间
    SELL_WEIGHT = 1.3  # 优先卖给格子被部分占用的
    SELL_DEBUFF = 0.8  # 非 7 卖给89的惩罚
    CONSERVATIVE = 1  # 保守程度 最后时刻要不要操作
    BUY_WEIGHT = [1]*4+[1]*3+[1]  # 购买优先级，优先购买高级商品
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

        self._buy_workstand_ori = None
        self._sell_cell_ori = None
        self._type_equal_ori = None
        self._buy_workstand_dynamic = None
        self._sell_cell_dynamic = None
        self._type_equal_dynamic = None
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

        # 原始工作台购买价格
        self._buy_workstand_ori = np.array(
            [self._workstands.product_workstand_dict[key][2] for key in self._workstands.product_workstand_dict])

        # 原始格子收购价格价格
        self._sell_cell_ori = np.array(
            [self._workstands.receive_cell_dict[key][2] for key in self._workstands.receive_cell_dict])

        # 动态采购出售价格初始化，生成规模一致的数组
        self._buy_workstand_dynamic = copy.deepcopy(self._buy_workstand_ori)
        self._sell_cell_dynamic = copy.deepcopy(self._sell_cell_ori)
        # 工作台生产物品类型
        type_workstand = np.array(
            [self._workstands.product_workstand_dict[key][3] for key in self._workstands.product_workstand_dict])

        # 格子收购物品类型
        type_cell = np.array([self._workstands.receive_cell_dict[key][3]
                             for key in self._workstands.receive_cell_dict])

        # 做差 0表示生产收购一致
        self._type_equal_ori = type_cell.reshape(
            1, -1) - type_workstand.reshape(-1, 1)

    def cal_profit_workstand2cell(self, frame_id_in):

        for idx_robot in range(4):
            # 计算格子收购价
            # 如果此格子有物品，其收购价设置为0
            # 如果邻居格子有物品，提高提高此格子预估收购价，促进合成
            for key_cell in range(self._workstands.count_cell):
                time_arrive = self._dis_robot2cell[idx_robot, key_cell] / 5.5
                # 此cell对应的接收产品与工作台id
                idx_workstand, material_receive = self._workstands.get_id_workstand_of_cell(
                    key_cell)

                # 获取工作台id的原料格状态
                _, _, material, _ = self._workstands.get_workstand_status(
                    idx_workstand)

                if int(material) & (1 << material_receive):
                    # 此格子已有物品
                    waiting_time = int(self._workstands.get_status(
                        feature_waiting_time_w, idx_workstand))
                    if waiting_time in [-1, 0]:
                        # 不在生产 或 阻塞
                        self._sell_cell_dynamic[key_cell] = -1000000000  # 负无穷
                    else:
                        # 在生产中
                        self._sell_cell_dynamic[key_cell] = -1000000000  # 负无穷
                        # self._sell_cell_dynamic[key_cell] = self._sell_cell_ori[key_cell] * 0.8
                else:
                    # 此格子没有物品
                    if int(material):
                        # 邻居格子有物品 此格子无物品
                        self._sell_cell_dynamic[key_cell] = self._sell_cell_ori[key_cell] * (
                            1 + 0.2 * count_ones(int(material))) ** 2

                    else:
                        # 都没有 不动
                        self._sell_cell_dynamic[key_cell] = self._sell_cell_ori[key_cell]

            # 计算工作台采购价
            # 如果此工作台产出格子没有物品，其收购价设置为Inf
            # 如果其自带格子有物品，适当降低采购价，防止阻塞
            for key_workstand in range(self._workstands.count):
                time_arrive = self._dis_robot2workstand[idx_robot,
                                                        key_workstand] / 5.5

                # 获取工作台id的原料格状态 产品格状态
                # _, _, material, product_status = self._workstands.get_workstand_status(idx_workstand)
                _, _, material, product_status = self._workstands.get_workstand_status(
                    key_workstand)
                material_count = count_ones(material)

                if int(product_status):
                    # 产品格已有物品
                    self._buy_workstand_dynamic[key_workstand] = self._buy_workstand_ori[key_workstand] * (
                        1 - 0.5 * material_count) ** 2  # 负无穷
                else:
                    # 产品格没有物品

                    if int(self._workstands.get_status(feature_waiting_time_w, key_workstand)) == -1:
                        # 没在生产
                        # 收购价设置为正无穷
                        self._buy_workstand_dynamic[key_workstand] = 1000000

                    elif int(self._workstands.get_status(feature_waiting_time_w, key_workstand)) <= 50 * 2:
                        self._buy_workstand_dynamic[key_workstand] = self._buy_workstand_ori[key_workstand]

                if int(self._workstands.get_status(feature_num_type_w, key_workstand)) in [1, 2, 3] and frame_id_in < 52:
                    self._buy_workstand_dynamic[key_workstand] = self._buy_workstand_ori[key_workstand]
            # 广播计算利润
            temp_profit_estimation = self._sell_cell_dynamic.reshape(
                1, -1) - self._buy_workstand_dynamic.reshape(-1, 1)

            # 将不符合采购出售的匹配类型交易利润设置为-inf
            temp_profit_estimation[self._type_equal_ori != 0] = -100000

            # 广播给4个机器人
            self._profit_estimation[idx_robot, :, :] = temp_profit_estimation
        pass

    def cal_dis_robot2workstand2cell(self):
        loc_robots = self._robots.get_loc(-1)
        loc_cell = np.array(
            [[self._workstands.receive_cell_dict[key][1], self._workstands.receive_cell_dict[key][0]] for key in
             self._workstands.receive_cell_dict])

        self._delta_x_r2c, self._delta_y_r2c, self._dis_robot2cell = get_dx_dy_d(
            loc_robots, loc_cell)

        self._dis_robot2workstand2cell = self._dis_robot2workstand[:, :, np.newaxis] + self._dis_cell2workstand.T[
            np.newaxis, :, :]

    def cal_cooldown(self):
        # 计算机器人-工作台(所有工作台)的冷却时间以及工作台-格子(所有格子)的冷却时间
        # 分两段计算 因为移动过程也是两段，要分两段取max

        # 机器人到工作台
        # 等待工作台的冷却时间 工作台广播
        for key_workstand in range(self._workstands.count):
            idx_workstand = key_workstand
            if int(self._workstands.get_status(feature_product_state_w, idx_workstand)) == 1:
                # 产品格有产品
                waiting_time = 0
            elif int(self._workstands.get_status(feature_waiting_time_w, idx_workstand)) == -1:
                # 没产品且不在生产中
                waiting_time = 10000
            else:
                waiting_time = int(self._workstands.get_status(
                    feature_waiting_time_w, idx_workstand))

            self._cooldown_robot2workstand[:, key_workstand] = waiting_time

        # 工作台到格子
        # 等待格子的冷却时间 格子广播
        for key_cell in range(self._workstands.count_cell):
            # 获取此格子对应的工作台id和收购产品类型
            idx_workstand, material_receive = self._workstands.get_id_workstand_of_cell(
                key_cell)

            # 获取工作台id的原料格状态
            _, _, material, _ = self._workstands.get_workstand_status(
                idx_workstand)
            if int(material) & (1 << material_receive):
                # 这个盒子已有物品
                waiting_time = int(self._workstands._workstand[idx_workstand,
                                                           feature_waiting_time_w])
                if waiting_time <= 0:
                    # -1 没在生产，不是冷却时间为-1
                    # 0 在阻塞，不是冷却时间为0
                    # workstand_type= self._workstands.get_status(feature_num_type_w, idx_workstand)
                    # waiting_time = cool_down_time_max[workstand_type]
                    waiting_time = 100000  # 冷却时间设为无穷

            else:
                # 还可以放物品
                waiting_time = 0
            self._cooldown_workstand2cell[:, key_cell] = waiting_time

    def cal_time(self):
        time_robot2workstand = np.maximum(
            self._dis_robot2workstand / 5.9 * 50, self._cooldown_robot2workstand)
        time_workstand2cell = np.maximum(
            self._dis_cell2workstand.T / 5.9 * 50, self._cooldown_workstand2cell)
        self._time_robot2workstand2cell = time_robot2workstand[:,
                                                               :, np.newaxis] + time_workstand2cell[np.newaxis, :, :]

    def cal_profit_rate(self):
        self._profit_rate_estimation = self._profit_estimation / \
            self._time_robot2workstand2cell

    def select(self):
        self._index_tran_r = np.where(self._robot_unlock)[0]
        self._index_tran_w = np.where(self._product_workstand_unlock)[0]
        self._index_tran_c = np.where(self._receive_cell_unlock)[0]
        temp = self._profit_rate_estimation[self._robot_unlock, :, :]
        temp = temp[:, self._product_workstand_unlock, :]
        profit_rate_estimation = temp[:, :, self._receive_cell_unlock]

        # 未被锁定的利率
        if self._robot_unlock[:].any() and (profit_rate_estimation > 0).any():
            r_id, w_id, c_id = np.unravel_index(
                np.argmax(profit_rate_estimation, axis=None), profit_rate_estimation.shape)
            r_id = self._index_tran_r[r_id]
            w_id = self._index_tran_w[w_id]
            c_id = self._index_tran_c[c_id]
            self._robots.set_status_item(feature_target_buy_r, r_id, w_id)
            self._robots.set_status_item(feature_target_sell_r, r_id, c_id)
            self._robot_unlock[r_id] = False

            if not int(self._workstands.get_status(feature_num_type_w, w_id)) in [1,2,3]:
                self._product_workstand_unlock[w_id] = False
            self._receive_cell_unlock[c_id] = False
            return True
        else:
            return False

    def change(self, idx_robot):
        # 机器人和原目标格子以及对应的工作台
        target_sell_cell = int(self._robots.get_status(
            feature_target_sell_r, idx_robot))
        target_workstand, _ = self._workstands.get_id_workstand_of_cell(
            target_sell_cell)
        target_workstand = int(target_workstand)

        material_carry = int(self._robots.get_status(
            feature_materials_r, idx_robot))
        # 选择距离当前工作台最近 的 且接收相同物品的格子
        min_dis = 10000
        idx_new_cell = -1
        for key_cell in range(self._workstands.count_cell):
            try_workstand, _ = self._workstands.get_id_workstand_of_cell(
                target_sell_cell)
            try_workstand = int(try_workstand)
            _, _, material, _ = self._workstands.get_workstand_status(
                try_workstand)

            # 格子里有物品
            try_flag = int(material) & (1 << material_carry) == 0
            if try_flag and self._dis_cell2workstand[key_cell, target_workstand] < min_dis and self._receive_cell_unlock[key_cell]:
                min_dis = self._dis_cell2workstand[key_cell, target_workstand]
                idx_new_cell = key_cell

        self._robots.set_status_item(
            feature_target_sell_r, idx_robot, idx_new_cell)
        new_workstand, _ = self._workstands.get_id_workstand_of_cell(
            idx_new_cell)
        self._robots.set_status_item(feature_target_r, idx_robot, new_workstand)

        self._receive_cell_unlock[idx_new_cell] = False

        # 释放出现异常无法卖出的格子
        self._receive_cell_unlock[target_sell_cell] = True

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

    def get_time_rww(self, idx_robot, idx_workstand1, idx_workstand2):
        dis_r2w1 = self._dis_robot2workstand[idx_robot, idx_workstand1]
        dis_w12w2 = self._dis_workstand2workstand[idx_workstand1, idx_workstand2]

        theta_r2w1 = self.get_delta_theta(idx_robot)
        theta_w12w2 = self.get_delta_theta2target(idx_robot, idx_workstand1, idx_workstand2)

        return compute_time_to_arrive(dis_r2w1, theta_r2w1), compute_time_to_arrive(theta_w12w2, theta_r2w1)


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

    def wait_other_buy(self, idx_robot, distance_r2w):
        # 当自己准备买时 如果目标格子有物品且工作台有产出尚未取出 需变更目标坐标等待
        flag = False
        target_sell_cell = int(self._robots.get_status(
            feature_target_sell_r, idx_robot))
        target_workstand, _ = self._workstands.get_id_workstand_of_cell(
            target_sell_cell)
        target_workstand = int(target_workstand)
        workstand_type, _, material, _ = map(
            int, self._workstands.get_workstand_status(target_workstand))
        material_type = int(self._robots.get_status(
            feature_materials_r, idx_robot))
        if (material & 1 << material_type) and self._workstands.get_status(feature_product_state_w, target_workstand) == 1 and distance_r2w < 4:
            flag = True
        return flag


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

    def move2loc_new_bck(self, idx_robot):
        # 输入控制机器人编号 目标工作台编号 期望速度
        # 结合人工势场计算速度
        idx_target = self._robots.get_status(feature_target_r, idx_robot)

        desired_theta, distance_r2w, near_flag = self.calculate_potential_field(
            idx_robot, idx_target)

        # 比例控制 追踪目标方向
        # 计算相差方向 P
        now_theta = self._robots.get_status(feature_theta_r, idx_robot)
        # now_theta = math.atan2(self._robots.get_status(feature_line_velo_y_r, idx_robot), self._robots.get_status(feature_line_velo_x_r, idx_robot))
        now_ang_velo = self._robots.get_status(feature_ang_velo_r, idx_robot)
        delta_theta = desired_theta - now_theta
        delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi
        k_r = 10
        n_r = 1
        k_s = 10
        n_s = 1.5

        # self._robots.rotate(idx_robot, delta_theta * k_r)
        if not near_flag == -1 and abs(delta_theta) < math.pi / 3:  #False:  #
            # 应该避让时
            speed = -1
            self._robots.forward(idx_robot, speed)
            # delta_theta += 0.7
            self._robots.rotate(idx_robot, delta_theta * k_r)
        else:
            d_far = 11
            d_near = 5
            d_daoche = 3

            ang_large = math.pi * 0.9
            ang_small = math.pi * 0.1
            if abs(delta_theta) < ang_small:
                # 面对目标方向
                self._robots.rotate(
                    idx_robot, sign_pow(delta_theta, n_r) * k_r)
                self._robots.forward(idx_robot, distance_r2w ** n_s * k_s)
            elif abs(delta_theta) < ang_large:
                if distance_r2w > d_far:
                    # 距离远且不面对目标方向

                    # 边开边转（有希望开近的时候面对目标）
                    self._robots.rotate(
                        idx_robot, sign_pow(delta_theta, n_r) * k_r)
                    self._robots.forward(idx_robot, distance_r2w ** n_s * k_s)
                elif distance_r2w > d_near:
                    # 距离适中且不面对目标方向

                    # 低速前进地转（防止凑近之后绕圈）
                    self._robots.rotate(
                        idx_robot, sign_pow(delta_theta, n_r) * k_r)
                    self._robots.forward(idx_robot, 2)
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
                    # continue
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

    def control2(self, frame_id: int):
        self.cal_dis_robot2workstand()
        self.cal_dis_robot2robot()

        # 计算移动耗时
        self.cal_dis_robot2workstand2cell()

        # 计算冷却耗时
        self.cal_cooldown()

        # 计算总耗时
        self.cal_time()

        # 计算利润
        self.cal_profit_workstand2cell(frame_id)

        # 计算利率 利润 / 耗时
        self.cal_profit_rate()

        # 从未被锁定的组合中依次选取最高
        self.select()
        self.select()
        self.select()
        self.select()

        for idx_robot in range(4):
            robot_status = int(self._robots.get_status(
                feature_status_r, idx_robot))
            if robot_status == RobotGroup.FREE_STATUS:
                # 【空闲】

                # 判定目标买点是否合法
                target_buy = int(self._robots.get_status(
                    feature_target_buy_r, idx_robot))
                # 将目标买点设为目标点
                if not target_buy == -1:
                    self._robots.set_status_item(
                        feature_target_r, idx_robot, target_buy)
                    # 切换状态为【购买途中】
                    self._robots.set_status_item(
                        feature_status_r, idx_robot, RobotGroup.MOVE_TO_BUY_STATUS)
                continue
            elif robot_status == RobotGroup.MOVE_TO_BUY_STATUS:
                # 【购买途中】

                self.move2loc_new(idx_robot)

                target_workstand = int(self._robots.get_status(
                    feature_target_buy_r, idx_robot))
                product_status = int(self._workstands.get_status(
                    feature_product_state_w, target_workstand))
                # 判定是否进入交互范围
                if self._robots.get_status(feature_workstand_id_r, idx_robot) == self._robots.get_status(
                        feature_target_r, idx_robot) and product_status == 1:
                    print("buy", idx_robot)

                    self._robots.set_status_item(
                        feature_status_r, idx_robot, RobotGroup.WAIT_TO_BUY_STATUS)  # 切换为 【买后验证】
                continue

            elif robot_status == RobotGroup.WAIT_TO_BUY_STATUS:
                # 【买后验证】
                if int(self._robots.get_status(feature_materials_r, idx_robot)) > 0:
                    # 确认成功买入

                    # 目标工作台
                    target_workstand = int(self._robots.get_status(
                        feature_target_buy_r, idx_robot))

                    # 目标格子
                    target_sell_cell = int(self._robots.get_status(
                        feature_target_sell_r, idx_robot))

                    # 目标格子对应的工作台
                    target_sell_workstand, _ = self._workstands.get_id_workstand_of_cell(
                        target_sell_cell)
                    target_sell_workstand = int(target_sell_workstand)

                    # 解锁这个工作台
                    self._product_workstand_unlock[target_workstand] = True

                    self._robots.set_status_item(
                        feature_target_r, idx_robot, target_sell_workstand)  # 更新目标到卖出地点
                    self._robots.set_status_item(
                        feature_buy_count, idx_robot, 0)
                    self._robots.set_status_item(
                        feature_status_r, idx_robot, RobotGroup.MOVE_TO_SELL_STATUS)  # 切换为 【出售途中】

                else:
                    self._robots.set_status_item(feature_buy_count, idx_robot, self._robots.get_status(
                        feature_buy_count, idx_robot) + 1)

                    if self._robots.get_status(feature_buy_count, idx_robot) < 2:
                        # 两次内购买失败说明位置不对，切换为 【购买途中】
                        self._robots.set_status_item(
                            feature_status_r, idx_robot, RobotGroup.MOVE_TO_BUY_STATUS)
                    else:
                        # 超过两次给购买失败
                        # 重新规划
                        target_sell_cell = int(self._robots.get_status(
                            feature_target_sell_r, idx_robot))
                        target_workstand, _ = self._workstands.get_id_workstand_of_cell(
                            target_sell_cell)
                        target_workstand = int(target_workstand)
                        workstand_type, _, material, _ = map(
                            int, self._workstands.get_workstand_status(target_workstand))
                        material_type = int(self._robots.get_status(
                            feature_materials_r, idx_robot))

                        # 解锁格子
                        self._receive_cell_unlock[target_sell_cell] = True
                        # 解锁工作台
                        self._product_workstand_unlock[target_workstand] = True
                        self._robot_unlock[idx_robot] = True  # 解锁机器人

                        self._robots.set_status_item(feature_target_r, -1)

                        self._robots.set_status_item(
                            feature_status_r, idx_robot, RobotGroup.FREE_STATUS)  # 切换为空闲
                        raise Exception('Buy Many times')
                continue

            elif robot_status == RobotGroup.MOVE_TO_SELL_STATUS:
                # 【出售途中】
                # 移动
                # 判断距离是否够近
                self.move2loc_new(idx_robot)

                target_sell_cell = int(self._robots.get_status(
                    feature_target_sell_r, idx_robot))
                target_workstand, _ = self._workstands.get_id_workstand_of_cell(
                    target_sell_cell)
                target_workstand = int(target_workstand)
                workstand_type, _, material, _ = map(
                    int, self._workstands.get_workstand_status(target_workstand))
                material_type = int(self._robots.get_status(
                    feature_materials_r, idx_robot))

                if idx_robot == 3:
                    aaaaaaaaa = 10000000000000
                # 判定是否进入交互范围 且格子内无材料
                if self._robots.get_status(feature_workstand_id_r, idx_robot) == self._robots.get_status(
                        feature_target_r, idx_robot) and (material & 1 << material_type) == 0:
                    print("sell", idx_robot)

                    self._robots.set_status_item(
                        feature_status_r, idx_robot, RobotGroup.WAIT_TO_SELL_STATUS)  # 切换为 【卖后验证】
                    # logging.debug(f"{idx_robot}->ready to sell")

                continue

            elif robot_status == RobotGroup.WAIT_TO_SELL_STATUS:
                # 【卖后验证】

                if int(self._robots.get_status(feature_materials_r, idx_robot)) == 0:
                    # 成功卖出
                    target_sell_cell = int(self._robots.get_status(
                        feature_target_sell_r, idx_robot))
                    self._receive_cell_unlock[target_sell_cell] = True  # 解锁格子
                    self._robot_unlock[idx_robot] = True  # 解锁机器人
                    self._robots.set_status_item(
                        feature_status_r, idx_robot, RobotGroup.FREE_STATUS)  # 切换为空闲
                    self._robots.set_status_item(
                        feature_sell_count, idx_robot, 0)
                    # logging.debug(f"{idx_robot}->wait")

                else:
                    # 卖出失败

                    self._robots.set_status_item(feature_sell_count, idx_robot,
                                                 self._robots.get_status(feature_sell_count, idx_robot) + 1)
                    if self._robots.get_status(feature_sell_count, idx_robot) < 2:
                        # 两次内卖出失败说明位置不对，切换为 【卖出途中】
                        self._robots.set_status_item(
                            feature_status_r, idx_robot, RobotGroup.MOVE_TO_SELL_STATUS)
                    else:
                        # 重新找卖家
                        self.change(idx_robot)
                        # 变更买家，切换为 【卖出途中】
                        self._robots.set_status_item(
                            feature_status_r, idx_robot, RobotGroup.MOVE_TO_SELL_STATUS)
                        # raise Exception('Sell Many times')
                continue