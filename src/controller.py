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
DIS_1 = 1.5
VELO_1 = 2
MOVE_SPEED = 1 / 4 * 50  # 估算移动时间
MAX_WAIT = 3 * 50  # 最大等待时间
SELL_WEIGHT = 1.2  # 优先卖给格子被部分占用的
# 人工势场常熟
ETA = 3000  # 调整斥力大小的常数
GAMMA = 10  # 调整吸引力大小的常数
RADIUS = 3  # 定义斥力半径范围
BUY_WEIGHT = [1] * 4 + [1] * 3 + [1]  # 购买优先级，优先购买高级商品
# 测试
DEBUG = False

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


class Controller:
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
        self._cooldown_workstand2cell = np.zeros((self._workstands.count, self._workstands.count_cell))
        self._time_robot2workstand2cell = np.zeros((4, self._workstands.count, self._workstands.count_cell))
        self._product_workstand_unlock = np.array([True] * workstands.count, dtype=np.bool)
        self._receive_cell_unlock = np.array([True] * workstands.count_cell, dtype=np.bool)
        self._robot_unlock = np.array([True] * 4, dtype=np.bool)
        self._profit_estimation = np.zeros((4, self._workstands.count, self._workstands.count_cell))
        self._profit_rate_estimation = np.zeros((4, self._workstands.count, self._workstands.count_cell))

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

    def cal_dis_robot2robot(self):
        # 计算所有机器人两两之间的距离 向量化 每来一帧调用一次
        # 距离表存在类变量中
        # 通过get_dis_robot2robot(self, idx_robot, idx_workstand)调用
        loc_robots1 = self._robots.get_loc(-1)
        loc_robots2 = self._robots.get_loc(-1)
        self._delta_x_r2r, self._delta_y_r2r, self._dis_robot2robot = get_dx_dy_d(loc_robots1, loc_robots2)

    def cal_dis_robot2workstand(self):
        # 计算所有机器人到所有工作站的距离 向量化 每来一帧调用一次
        # 距离表存在类变量中
        # 通过get_dis_robot2workstand(self, idx_robot, idx_workstand)调用
        loc_robots = self._robots.get_loc(-1)
        loc_workstands = self._workstands.get_loc(-1)
        self._delta_x_r2w, self._delta_y_r2w, self._dis_robot2workstand = get_dx_dy_d(loc_robots, loc_workstands)

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
        sell_cell = np.array([self._workstands.receive_cell_dict[key][2] for key in self._workstands.receive_cell_dict])

        self._delta_x_c2w, self._delta_y_c2w, self._dis_cell2workstand = get_dx_dy_d(loc_cell, loc_workstand)

        # 原始工作台购买价格
        self._buy_workstand_ori = np.array(
            [self._workstands.product_workstand_dict[key][2] for key in self._workstands.product_workstand_dict])

        # 原始格子收购价格价格
        self._sell_cell_ori = np.array([self._workstands.receive_cell_dict[key][2] for key in self._workstands.receive_cell_dict])

        # 动态采购出售价格初始化，生成规模一致的数组
        self._buy_workstand_dynamic = copy.deepcopy(self._buy_workstand_ori)
        self._sell_cell_dynamic = copy.deepcopy(self._sell_cell_ori)
        # 工作台生产物品类型
        type_workstand = np.array(
            [self._workstands.product_workstand_dict[key][3] for key in self._workstands.product_workstand_dict])

        # 格子收购物品类型
        type_cell = np.array([self._workstands.receive_cell_dict[key][3] for key in self._workstands.receive_cell_dict])

        # 做差 0表示生产收购一致
        self._type_equal_ori = type_cell.reshape(1, -1) - type_workstand.reshape(-1, 1)

    def cal_profit_workstand2cell(self, frame_id_in):

        # 计算格子收购价
        # 如果此格子有物品，其收购价设置为0
        # 如果邻居格子有物品，提高提高此格子预估收购价，促进合成
        for key_cell in range(self._workstands.count_cell):
            
            # 此cell对应的接收产品与工作台id
            idx_workstand, material_receive = self._workstands.get_id_workstand_of_cell(key_cell)

            # 获取工作台id的原料格状态
            _, _, material, _ = self._workstands.get_workstand_status(idx_workstand)

            if int(material) & (1 << material_receive):
                # 此格子已有物品
                self._sell_cell_dynamic[key_cell] = -1000000  # 负无穷
            else:
                # 此格子没有物品
                if int(material):
                    # 邻居格子有物品 此格子无物品
                    self._sell_cell_dynamic[key_cell] = self._sell_cell_ori[key_cell] * (1 + 0.1 * count_ones(int(material)))
                else:
                    # 都没有 不动
                    self._sell_cell_dynamic[key_cell] = self._sell_cell_ori[key_cell]
            

        # 计算工作台采购价
        # 如果此工作台产出格子没有物品，其收购价设置为Inf
        # 如果其自带格子有物品，适当降低采购价，防止阻塞
        for key_workstand in range(self._workstands.count):

            # 获取工作台id的原料格状态 产品格状态
            _, _, material, product_status = self._workstands.get_workstand_status(idx_workstand)
            material_count = count_ones(material)

            if int(product_status):
                # 产品格已有物品
                self._buy_workstand_dynamic[key_workstand] = self._buy_workstand_ori[key_workstand] * (1 - 0.1 * material_count)  # 负无穷
            else:
                # 产品格没有物品
                self._buy_workstand_dynamic[key_workstand] = 1000000  # 收购价设置为正无穷

            if int(self._workstands.get_status(feature_num_type_w, key_workstand)) in [1, 2, 3] and frame_id_in < 52:
                self._buy_workstand_dynamic[key_workstand] = self._buy_workstand_ori[key_workstand]
        # 广播计算利润
        temp_profit_estimation = self._sell_cell_dynamic.reshape(1, -1) - self._buy_workstand_dynamic.reshape(-1, 1)

        # 将不符合采购出售的匹配类型交易利润设置为-inf
        temp_profit_estimation[self._type_equal_ori != 0] = -100000
        
        # 广播给4个机器人
        for i in range(4):
            self._profit_estimation[i, :, :] = temp_profit_estimation
        pass

    def cal_dis_robot2workstand2cell(self):
        loc_robots = self._robots.get_loc(-1)
        loc_cell = np.array(
            [[self._workstands.receive_cell_dict[key][1], self._workstands.receive_cell_dict[key][0]] for key in
             self._workstands.receive_cell_dict])

        self._delta_x_r2c, self._delta_y_r2c, self._dis_robot2cell = get_dx_dy_d(loc_robots, loc_cell)

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
                waiting_time = int(self._workstands.get_status(feature_waiting_time_w, idx_workstand))

            self._cooldown_robot2workstand[:, key_workstand] = waiting_time

        # 工作台到格子
        # 等待格子的冷却时间 格子广播
        for key_cell in range(self._workstands.count_cell):
            # 获取此格子对应的工作台id和收购产品类型
            idx_workstand, material_receive = self._workstands.get_id_workstand_of_cell(key_cell)

            # 获取工作台id的原料格状态
            _, _, material, _ = self._workstands.get_workstand_status(idx_workstand)
            if int(material) & (1 << material_receive):
                # 这个盒子已有物品
                waiting_time = self._workstands._workstand[idx_workstand, feature_waiting_time_w]
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
        time_robot2workstand = np.maximum(self._dis_robot2workstand / 5.9 * 50, self._cooldown_robot2workstand)
        time_workstand2cell = np.maximum(self._dis_cell2workstand.T / 5.9 * 50, self._cooldown_workstand2cell)
        self._time_robot2workstand2cell = time_robot2workstand[:, :, np.newaxis] + time_workstand2cell[np.newaxis, :, :]

    def cal_profit_rate(self):
        self._profit_rate_estimation = self._profit_estimation / self._time_robot2workstand2cell

    def select(self):
        self._index_tran_r = np.where(self._robot_unlock)[0]
        self._index_tran_w = np.where(self._product_workstand_unlock)[0]
        self._index_tran_c = np.where(self._receive_cell_unlock)[0]
        temp = self._profit_rate_estimation[self._robot_unlock, :, :]
        temp = temp[:, self._product_workstand_unlock, :]
        profit_rate_estimation = temp[:, :, self._receive_cell_unlock]

        # 未被锁定的利率
        if self._robot_unlock[:].any() and (profit_rate_estimation > 0).any():
            r_id, w_id, c_id = np.unravel_index(np.argmax(profit_rate_estimation, axis=None), profit_rate_estimation.shape)
            r_id = self._index_tran_r[r_id]
            w_id = self._index_tran_w[w_id]
            c_id = self._index_tran_c[c_id]
            if w_id == 8:
                aaaaaa = 0
            self._robots.set_status_item(feature_target_buy_r, r_id, w_id)
            self._robots.set_status_item(feature_target_sell_r, r_id, c_id)
            self._robot_unlock[r_id] = False
            self._product_workstand_unlock[w_id] = False
            self._receive_cell_unlock[c_id] = False
            return True
        else:
            return False


    def change(self, idx_robot):
        # 机器人和原目标格子以及对应的工作台
        target_sell_cell = int(self._robots.get_status(feature_target_sell_r, idx_robot))
        target_workstand, _ = self._workstands.get_id_workstand_of_cell(target_sell_cell)
        target_workstand = int(target_workstand)

        material_carry = int(self._robots.get_status(feature_materials_r, idx_robot))
        # 选择距离当前工作台最近 的 且接收相同物品的格子
        min_dis = 10000
        idx_new_cell = -1
        for key_cell in range(self._workstands.count_cell):
            try_workstand, _ = self._workstands.get_id_workstand_of_cell(target_sell_cell)
            try_workstand = int(try_workstand)
            _, _, material, _ = self._workstands.get_workstand_status(try_workstand)

            # 格子里有物品
            try_flag = int(material) & (1 << material_carry)
            if not try_flag and self._dis_cell2workstand[key_cell, target_workstand] < min_dis:
                min_dis = self._dis_cell2workstand[key_cell, target_workstand]
                idx_new_cell = key_cell
        if idx_new_cell == 8:
            aaaaaaa=1000
        self._robots.set_status_item(feature_target_sell_r, idx_robot, idx_new_cell)
        self._robots.set_status_item(feature_target_r, idx_robot, idx_new_cell)

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

    def calculate_potential_field(self, idx_robot, idx_workstand):
        # 计算位于current_pos处的机器人的势能场
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

                theta_other = self._robots.get_status(
                    feature_theta_r, idx_other)

                # 各向异性判断是否会产生潜在的碰撞事件
                dircos_other = np.array(
                    [math.cos(theta_other), math.sin(theta_other)])
                ang_robot = math.acos(
                    np.dot(dircos_robot, np.array([-dx_robot, -dy_robot])))

                ang_other = math.acos(
                    np.dot(dircos_other, np.array([dx_robot, dy_robot])))

                # 如果机器人之间的距离小于一定半径范围，则计算斥力
                if distance_robot < RADIUS and (ang_robot < math.pi * 0.2 or ang_other < math.pi * 0.2):
                    repulsive_force = 0.5 * idx_robot * ETA * \
                                      ((1.0 / distance_robot) - (1.0 / RADIUS)) ** 2
                    repulsive_field[0] -= repulsive_force * dx_robot
                    repulsive_field[1] -= repulsive_force * dy_robot

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
        return desired_angle, distance_r2w

    def move2loc(self, idx_robot, speed):
        # 输入控制机器人编号 目标工作台编号 期望速度
        # 结合人工势场计算速度
        idx_target = self._robots.get_status(feature_target_r, idx_robot)

        desired_theta, distance_r2w = self.calculate_potential_field(idx_robot, idx_target)

        # 比例控制 追踪目标方向
        # 计算相差方向 P
        now_theta = self._robots.get_status(feature_theta_r, idx_robot)
        now_ang_velo = self._robots.get_status(feature_ang_velo_r, idx_robot)
        delta_theta = desired_theta - now_theta
        delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi
        k = 10
        # self._robots.rotate(idx_robot, delta_theta * k)
        if abs(delta_theta) < math.pi * 0.8 or distance_r2w > 3:
            self._robots.rotate(idx_robot, delta_theta * k)
            if abs(delta_theta) < math.pi / 3:
                self._robots.forward(idx_robot, speed)
            else:
                self._robots.forward(idx_robot, speed*delta_theta * k*0.5)
        else:
            self._robots.rotate(idx_robot, -delta_theta * k)
            self._robots.forward(idx_robot, -speed)
        # if delta_theta > -0.9 * math.pi and desired_theta < 0.9 * math.pi:
        #     # 需要顺时针转动追踪目标方向
        #     self._robots.rotate(idx_robot, delta_theta * k)
        # elif abs(now_ang_velo) > 0.01:
        #     # 防止有转速时在小区间震荡
        #     # 按原转速冲过震荡区间
        #     self._robots.rotate(idx_robot, np.sign(now_ang_velo))
        # else:
        #     # 无转速按原策略
        #     self._robots.rotate(idx_robot, delta_theta * k)
        #
        # if abs(delta_theta) > math.pi / 3 and distance_r2w < 5:
        #     speed = abs(delta_theta) * k * 0.25
        #
        # self._robots.forward(idx_robot, speed)

    def get_time_rate(self, frame_sell: float) -> float:
        # 计算时间损失
        if frame_sell >= 9000:
            return 0.8
        sqrt_num = math.sqrt(1 - (1 - frame_sell / 9000) ** 2)
        return (1 - sqrt_num) * 0.2 + 0.8

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
            if frame_wait_buy > MAX_WAIT:
                continue
            frame_move_to_buy = self.get_dis_robot2workstand(
                idx_robot, idx_workstand) * MOVE_SPEED
            buy_weight = BUY_WEIGHT[workstand_type]
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
                if WORKSTAND_OUT[sell_type] and 1 << workstand_type & sell_material:
                    continue
                    # if sell_product_time == -1:  # 剩余生产时间为0，说明生产阻塞
                    #     continue
                    # else:
                    #     frame_wait_sell = sell_product_time
                frame_move_to_sell = self.get_dis_workstand2workstand(
                    idx_workstand, idx_worksand_to_sell) * MOVE_SPEED
                frame_buy = max(frame_move_to_buy,
                                frame_wait_buy)  # 购买时间
                # frame_sell = max(frame_move_to_sell,
                #                  frame_wait_sell - frame_buy)  # 出售时间
                total_frame = frame_buy + frame_move_to_sell  # 总时间
                if total_frame + frame_id > MATCH_FRAME:  # 完成这套动作就超时了
                    continue
                time_rate = self.get_time_rate(
                    frame_move_to_sell)  # 时间损耗
                sell_weight = SELL_WEIGHT if sell_material else 1
                radio = (
                                ITEMS_SELL[workstand_type] * time_rate - ITEMS_BUY[
                            workstand_type]) / total_frame * sell_weight * buy_weight
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
            self._workstands.set_product_pro(target_walkstand, 1)

            material_pro = int(
                self._workstands.get_material_pro(next_walkstand))
            workstand_types = int(
                self._workstands.get_workstand_status(target_walkstand)[0])
            self._workstands.set_material_pro(
                next_walkstand, material_pro + (1 << workstand_types))
            self._robots.set_status_item(
                feature_status_r, idx_robot, RobotGroup.MOVE_TO_BUY_STATUS)
            return True
        return False

    def control(self, frame_id: int):
        self.cal_dis_robot2workstand()
        self.cal_dis_robot2robot()

        self.cal_dis_robot2workstand2cell()
        idx_robot = 0
        while idx_robot < 4:
            robot_status = int(self._robots.get_status(
                feature_status_r, idx_robot))
            if robot_status == RobotGroup.FREE_STATUS:
                # 【空闲】执行调度策略
                if self.choise(frame_id, idx_robot):
                    continue
            elif robot_status == RobotGroup.MOVE_TO_BUY_STATUS:
                # 【购买途中】

                if self.get_dis_robot2workstand(idx_robot,
                                                self._robots.get_status(feature_target_r, idx_robot)) < DIS_1:
                    self.move2loc(idx_robot, VELO_1)
                else:
                    self.move2loc(idx_robot, 6)

                # 判定是否进入交互范围
                if self._robots.get_status(feature_workstand_id_r, idx_robot) == self._robots.get_status(
                        feature_target_r, idx_robot):
                    self._robots.set_status_item(
                        feature_status_r, idx_robot, RobotGroup.WAIT_TO_BUY_STATUS)  # 切换为 【等待购买】
                    continue
            elif robot_status == RobotGroup.WAIT_TO_BUY_STATUS:
                # 【等待购买】
                target_walkstand, next_walkstand = self._robots.robots_plan[idx_robot]
                product_status = int(
                    self._workstands.get_workstand_status(target_walkstand)[3])
                # 如果在等待，提前转向
                if product_status == 1:  # 这里判定是否生产完成可以购买 不是真的1
                    # 可以购买
                    if self._robots.buy(idx_robot):  # 防止购买失败
                        self._workstands.set_product_pro(
                            target_walkstand, 0)  # 取消预购
                        self._robots.set_status_item(
                            feature_target_r, idx_robot, next_walkstand)  # 更新目标到卖出地点
                        self._robots.set_status_item(
                            feature_status_r, idx_robot, RobotGroup.MOVE_TO_SELL_STATUS)  # 切换为 【出售途中】
                        # logging.debug(f"{idx_robot}->way to sell")
                        continue
                    else:
                        self._robots.set_status_item(
                            feature_status_r, idx_robot, RobotGroup.MOVE_TO_BUY_STATUS)  # 购买失败说明位置不对，切换为 【购买途中】
                        continue

            elif robot_status == RobotGroup.MOVE_TO_SELL_STATUS:
                # 【出售途中】
                # 移动
                # 判断距离是否够近
                if self.get_dis_robot2workstand(idx_robot,
                                                self._robots.get_status(feature_target_r, idx_robot)) < DIS_1:
                    self.move2loc(idx_robot, VELO_1)
                else:
                    self.move2loc(idx_robot, 6)

                # 判定是否进入交互范围
                if self._robots.get_status(feature_workstand_id_r, idx_robot) == self._robots.get_status(
                        feature_target_r, idx_robot):
                    self._robots.set_status_item(
                        feature_status_r, idx_robot, RobotGroup.WAIT_TO_SELL_STATUS)  # 切换为 【等待出售】
                    # logging.debug(f"{idx_robot}->ready to sell")
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
                        material_pro = int(
                            self._workstands.get_material_pro(target_walkstand))
                        self._workstands.set_material_pro(
                            target_walkstand, material_pro - (1 << material_type))
                        self._robots.set_status_item(
                            feature_status_r, idx_robot, RobotGroup.FREE_STATUS)  # 切换为空闲
                        # logging.debug(f"{idx_robot}->wait")
                        break  # 防止别人以它为目标 后面可以优化改为最后集中处理预售
                    else:
                        self._robots.set_status_item(
                            feature_status_r, idx_robot, RobotGroup.MOVE_TO_SELL_STATUS)  # 购买失败说明位置不对，切换为 【出售途中】
                        continue
            idx_robot += 1

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
                target_buy = int(self._robots.get_status(feature_target_buy_r, idx_robot))
                # 将目标买点设为目标点
                if not target_buy == -1:
                    self._robots.set_status_item(feature_target_r, idx_robot, target_buy)
                    # 切换状态为【购买途中】
                    self._robots.set_status_item(
                        feature_status_r, idx_robot, RobotGroup.MOVE_TO_BUY_STATUS)
                continue
            elif robot_status == RobotGroup.MOVE_TO_BUY_STATUS:
                # 【购买途中】
                if self.get_dis_robot2workstand(idx_robot,
                                                self._robots.get_status(feature_target_r, idx_robot)) < DIS_1:
                    self.move2loc(idx_robot, VELO_1)
                else:
                    self.move2loc(idx_robot, 6)

                target_workstand = int(self._robots.get_status(feature_target_buy_r, idx_robot))
                product_status = int(self._workstands.get_status(feature_product_state_w, target_workstand))
                # 判定是否进入交互范围
                if self._robots.get_status(feature_workstand_id_r, idx_robot) == self._robots.get_status(feature_target_r, idx_robot) and product_status == 1:

                    print("buy", idx_robot)

                    self._robots.set_status_item(
                        feature_status_r, idx_robot, RobotGroup.WAIT_TO_BUY_STATUS)  # 切换为 【买后验证】
                continue

            elif robot_status == RobotGroup.WAIT_TO_BUY_STATUS:
                # 【买后验证】
                if int(self._robots.get_status(feature_materials_r, idx_robot)) > 0:
                    # 确认成功买入

                    # 目标工作台
                    target_workstand = int(self._robots.get_status(feature_target_buy_r, idx_robot))

                    # 目标格子
                    target_sell_cell = int(self._robots.get_status(feature_target_sell_r, idx_robot))

                    # 目标格子对应的工作台
                    target_sell_workstand, _ = self._workstands.get_id_workstand_of_cell(target_sell_cell)
                    target_sell_workstand = int(target_sell_workstand)

                    self._product_workstand_unlock[target_workstand] = True  # 解锁这个工作台


                    self._robots.set_status_item(
                        feature_target_r, idx_robot, target_sell_workstand)  # 更新目标到卖出地点
                    self._robots.set_status_item(feature_buy_count, idx_robot, 0)
                    self._robots.set_status_item(
                        feature_status_r, idx_robot, RobotGroup.MOVE_TO_SELL_STATUS)  # 切换为 【出售途中】

                else:
                    self._robots.set_status_item(feature_buy_count,idx_robot, self._robots.get_status(feature_buy_count, idx_robot) + 1)

                    if self._robots.get_status(feature_buy_count, idx_robot) < 2:
                        # 两次内购买失败说明位置不对，切换为 【购买途中】
                        self._robots.set_status_item(
                            feature_status_r, idx_robot, RobotGroup.MOVE_TO_BUY_STATUS)
                    else:
                        # 超过两次给购买失败
                        # 重新规划
                        target_sell_cell = int(self._robots.get_status(feature_target_sell_r, idx_robot))
                        target_workstand, _ = self._workstands.get_id_workstand_of_cell(target_sell_cell)
                        target_workstand = int(target_workstand)
                        workstand_type, _, material, _ = map(
                            int, self._workstands.get_workstand_status(target_workstand))
                        material_type = int(self._robots.get_status(
                            feature_materials_r, idx_robot))
                        self._receive_cell_unlock[target_sell_cell] = True  # 解锁格子
                        self._product_workstand_unlock[target_workstand] = True  # 解锁工作台
                        self._robot_unlock[idx_robot] = True  # 解锁机器人
                        self._robots.set_status_item(
                            feature_status_r, idx_robot, RobotGroup.FREE_STATUS)  # 切换为空闲
                        raise Exception('Buy Many times')
                continue

            elif robot_status == RobotGroup.MOVE_TO_SELL_STATUS:
                # 【出售途中】
                # 移动
                # 判断距离是否够近
                if self.get_dis_robot2workstand(idx_robot,
                                                self._robots.get_status(feature_target_r, idx_robot)) < DIS_1:
                    self.move2loc(idx_robot, VELO_1)
                else:
                    self.move2loc(idx_robot, 6)

                target_sell_cell = int(self._robots.get_status(feature_target_sell_r, idx_robot))
                target_workstand, _ = self._workstands.get_id_workstand_of_cell(target_sell_cell)
                target_workstand = int(target_workstand)
                workstand_type, _, material, _ = map(
                    int, self._workstands.get_workstand_status(target_workstand))
                material_type = int(self._robots.get_status(
                    feature_materials_r, idx_robot))

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
                    target_sell_cell = int(self._robots.get_status(feature_target_sell_r, idx_robot))
                    self._receive_cell_unlock[target_sell_cell] = True  # 解锁格子
                    self._robot_unlock[idx_robot] = True  # 解锁机器人
                    self._robots.set_status_item(
                        feature_status_r, idx_robot, RobotGroup.FREE_STATUS)  # 切换为空闲
                    self._robots.set_status_item(feature_sell_count, idx_robot, 0)
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
                        raise Exception('Sell Many times')
                continue

                


