# -- coding: utf-8 --
# @Time : 2023/3/19 13:38
# @Author : Gao Peng
#!/bin/bash
# coding=utf-8
import sys
import copy
import time

import numpy as np
import math
import logging
import heapq
import logging

# 第一步，创建一个logger
logger = logging.getLogger()
logger.setLevel(logging.INFO)  # Log等级总开关  此时是INFO

# 第二步，创建一个handler，用于写入日志文件

# 第三步，再创建一个handler，用于输出到控制台
ch = logging.StreamHandler()
ch.setLevel(logging.INFO)   # 输出到console的log等级的开关

# 第四步，定义handler的输出格式（时间，文件，行数，错误级别，错误提示）
formatter = logging.Formatter("%(asctime)s - %(filename)s[line:%(lineno)d] - %(levelname)s: %(message)s")

ch.setFormatter(formatter)

# 第五步，将logger添加到handler里面
logger.addHandler(ch)
DIS_1 = 0.4
VELO_1 = 0.1
DISMAP = None  # 初始化时更新，记录任意两个工作台间的测算距离/帧数
ITEMS_BUY = [0, 3000, 4400, 5800, 15400, 17200, 19200, 76000]  # 每个物品的购买价
ITEMS_SELL = [0, 6000, 7600, 9200, 22500, 25000, 27500, 105000]
ITEMS_NEED = [[] for _ in range(8)]  # 记录收购每个商品的工作台编号
WORKSTAND_IN = {1: [], 2: [], 3: [], 4: [1, 2], 5: [1, 3],
                6: [2, 3], 7: [4, 5, 6], 8: [7], 9: list(range(1, 8))}
WORKSTAND_OUT = {i: i for i in range(1, 8)}
WORKSTAND_OUT[8] = None
WORKSTAND_OUT[9] = None
MOVE_SPEED = 8  # 估算移动时间
MAX_WAIT = 3 * 50  # 最大等待时间
DIS_TO_LOW_SPEED = 6
LOWEST_SPEED = 1
LOW_SPEED = 0.3

PROMATERIAL =  dict()
PROMATERIAL[4] =  ( 1 << 1) | (1 << 2)
PROMATERIAL[5] =  ( 1 << 1) | (1 << 3)
PROMATERIAL[6] =  ( 1 << 2) | (1 << 3)

# 人工势场常熟
ETA = 300  # 调整斥力大小的常数
GAMMA = 10  # 调整吸引力大小的常数
RADIUS = 3  # 定义斥力半径范围
INFINITE = 99999999
ALL_FRAME_COUNT = 3 * 60 * 50 # 总帧数
CURRENT_FRAME = 0 # 当前帧
FRAME_MUL = 1
# 全局变量 特证明命名的变量对应索引值 防止字典查询太慢
# _r结尾 机器人特征
feature_workstand_id_r = 0
feature_materials_r = 1
feature_time_value_r = 2
feature_collision_r = 3
feature_ang_velo_r = 4
feature_line_velo_x_r = 5
feature_line_velo_y_r = 6
feature_theta_r = 7
feature_x_r = 8
feature_y_r = 9
feature_status_r = 10  # 0 空闲, 1 购买途中, 2 等待购买, 3 出售途中, 4 等待出售
feature_target_r = 11
feature_target_theta_r = 12

# _w结尾 工作台特征
feature_num_type_w = 0
feature_x_w = 1
feature_y_w = 2
feature_waiting_time_w = 3
feature_raw_state_w = 4
feature_product_state_w = 5
feature_material_pro_w = 6
feature_product_pro_w = 7

class MaterialOKCount:
    """
    如果4,5,6类产品需要的原材料都准备好，则对应的count+1
    """
    def __init__(self):
        self.product2count = dict()
        self.product2count[4]=1
        self.product2count[5]=1
        self.product2count[6] = 1
        self.product2count[7]=1
        self.product2count[8] = 1

    def add(self,key,count=1):
        if count < 0:
            raise Exception
        if key > 8 or key < 4:
            return
        self.product2count[key] = self.product2count[key] + count

    def get_key_of_min_count(self):
        key = set()
        count = INFINITE
        for k,v in self.product2count.items():
            if v == count:
                key.add(k)
            elif v < count:
                count = v
                key.clear()
                key.add(k)
            else:
                pass
        return key

material_ok_count = MaterialOKCount()

def clrbit(x,y):
    return x & (~(1<<y))
def count_2(v):
    num = 0
    while v:
        num += v&1
        v >>= 1
    return num
class Task:
    def __init__(self,ratio,robot_idx, product_workstand_idx,sell_workstand_idx):
        self.__ratio = ratio
        self.__robot_idx = robot_idx
        self.__product_workstand_idx=product_workstand_idx
        self.__sell_workstand_idx=sell_workstand_idx

    def __lt__(self, other):
        if self.__ratio != other.get_ratio():
            return self.__ratio < other.get_ratio()
        return self.__robot_idx < other.get_robot_idx()

    def get_ratio(self):
        return self.__ratio

    def get_robot_idx(self):
        return self.__robot_idx

    def get_product_workstand_idx(self):
        return self.__product_workstand_idx

    def get_sell_workstand_idx(self):
        return self.__sell_workstand_idx

    def get_key(self):
        return str(self.get_sell_workstand_idx()) + str(self.get_product_workstand_idx())

def rate(price_sell):
    if price_sell > 9000:
        return 0.8
    return (1 - math.sqrt(1 - (1 - price_sell/9000)**2)) * 0.2 + 0.8

class Map:
    def __init__(self):
        # 工作台信息 array
        self._workstand = np.array([]).reshape(0, 8)
        self.count = 0

    def add_workstand(self, num_type, x, y):
        # 第一次读地图添加工作台
        # 类型 x y 剩余生产时间 原材料格状态 产品格状态 预售接口 预购接口
        new_info = np.array(
            [num_type, x, y, 0.0, 0.0, 0.0] + [0] * 2).reshape(1, 8)
        self._workstand = np.concatenate([self._workstand, new_info], axis=0)
        self.count += 1

    def update_platform(self, idx_platform, state_str):
        # 后面读地图 更新工作台状态
        # 输入：字符串

        # 字符串解析为变量
        str_list = state_str.split()
        float_list = [float(str_item) for str_item in str_list]
        # num_type, x, y, waiting_time, raw_state, product_state = float_list
        self._workstand[idx_platform, :6] = np.array(float_list)

    def set_material_pro(self, idx_workstand, value):
        # 预售接口 如果是89特殊处理
        self._workstand[idx_workstand, -2] = value

    def set_product_pro(self, idx_workstand, value):
        # 预购接口, 如果是123特殊处理
        self._workstand[int(idx_workstand), -1] = value

    def get_material_pro(self, idx_workstand):
        # 原料格预定状态, 防止有多个机器人将其作为出售目标
        return copy.deepcopy(self._workstand[idx_workstand, -2])

    def get_product_pro(self, idx_workstand):
        # 防止有多个机器人将其作为购买目标
        return copy.deepcopy(self._workstand[idx_workstand, -1])

    def get_workstand_status(self, idx_workstand):
        """
        workstand_type, product_time, material, product_status
        :param idx_workstand:
        :return:
        """
        workstand_type, product_time, material, product_status = self._workstand[idx_workstand, [
            0, 3, 4, 5]].tolist()
        return workstand_type, product_time, material, product_status

    def get_loc(self, idx_workstand):
        if idx_workstand == -1:
            return copy.deepcopy(self._workstand[:, [1, 2]])
        else:
            return copy.deepcopy(self._workstand[idx_workstand, [1, 2]])

    def __len__(self):
        return self.count


class RobotGroup:
    # 0 空闲, 1 购买途中, 2 等待购买, 3 出售途中, 4 等待出售
    FREE_STATUS = 0
    MOVE_TO_BUY_STATUS = 1
    WAIT_TO_BUY_STATUS = 2
    MOVE_TO_SELL_STATUS = 3
    WAIT_TO_SELL_STATUS = 4

    def __init__(self):
        self.group_info = np.zeros((4, 13))
        self.robots_plan = [[-1, -1] for _ in range(4)]  # 记录每个机器人的买和卖目标

    def add_init_location(self, idx_robot, x, y):
        # 判题器获取
        # 0-platform_id, 1-materials, 2-time_value_coef, 3-collision_value_coef,
        # 4-ang_velo, 5-line_velo_x, 6-line_velo_x, 7-theta, 8-x, 9-y

        # 自定义
        # 10-status, 11-target, 12-target_theta
        self.group_info[idx_robot, :] = np.array(
            [-1, x, y] + [0] * 10)

    def update_robot(self, id_robot, state_str):
        # 后面读地图 更新工作台状态
        # 输入：字符串

        # 字符串解析为变量
        str_list = state_str.split()
        float_list = [float(str_item) for str_item in str_list]
        self.group_info[id_robot, :10] = np.array(float_list)

    # 不可set的变量[0-9]
    def get_loc(self, idx_robot):
        if idx_robot == -1:
            return copy.deepcopy(self.group_info[:, [8, 9]])
        else:
            return copy.deepcopy(self.group_info[idx_robot, [8, 9]])

    # 自定义变量【10-12】
    def get_status(self, feature_id, idx_robot):
        # 获取指定机器人状态
        # idx_robot为-1表示获取所有机器人状态
        if idx_robot == -1:
            return copy.deepcopy(self.group_info[:, feature_id])
        else:
            return copy.deepcopy(self.group_info[idx_robot, feature_id])

    # 自定义变量【10-12】
    def set_status_item(self, feature_id, idx_robot, value):
        # 设定指定机器人状态
        self.group_info[idx_robot, feature_id] = value

    # 四个动作
    def forward(self, idx_robot, speed):
        '''
        设置前进速度，单位为米/秒。
        正数表示前进。
        负数表示后退。
        '''

        # # logging.info(f"forward {idx_robot} {speed}")
        print("forward", idx_robot, speed)

    def rotate(self, idx_robot, turn):
        '''
        设置旋转速度，单位为弧度/秒。
        负数表示顺时针旋转。
        正数表示逆时针旋转。
        '''
        # logging.info(f"rotate {idx_robot} {turn}")
        print("rotate", idx_robot, turn)

    def buy(self, idx_robot):
        '''
        购买当前工作台的物品，以输入数据的身处工作台 ID 为准。
        '''
        if self.get_status(feature_workstand_id_r, idx_robot) == self.get_status(feature_target_r, idx_robot):

            # logging.info(f"buy {idx_robot}")
            print("buy", idx_robot)
            return True
        else:
            return False

    def sell(self, idx_robot):
        '''
        出售物品给当前工作台，以输入数据的身处工作台 ID 为准。
        '''
        if self.get_status(feature_workstand_id_r, idx_robot) == self.get_status(feature_target_r, idx_robot):

            # logging.info(f"sell {idx_robot}")
            print("sell", idx_robot)
            return True
        else:
            return False

    def destroy(self, idx_robot):
        '''
        销毁物品。
        '''
        # logging.info("destroy", idx_robot)
        print("destroy", idx_robot)


class Controller:
    def __init__(self, robots: RobotGroup, workstands: Map):
        self._robots = robots
        self._workstands = workstands
        self._dis_robot2robot = None
        self._dis_robot2workstand = None
        self._dis_workstand2workstand = None
        self._delta_x_r2r = None
        self._delta_y_r2r = None
        self._delta_x_w2w = None
        self._delta_y_w2w = None
        self._delta_x_r2w = None
        self._delta_y_r2w = None
        self._idx_robox_2_task = dict()

    def cal_dis_robot2robot(self):
        # 计算所有机器人两两之间的距离 向量化 每来一帧调用一次
        # 距离表存在类变量中
        # 通过get_dis_robot2robot(self, idx_robot, idx_workstand)调用
        loc_robots1 = self._robots.get_loc(-1)
        loc_robots2 = self._robots.get_loc(-1)
        self._delta_x_r2r = loc_robots1[:, 0].reshape(-1, 1) - loc_robots2[:, 0].reshape(1, -1)
        self._delta_y_r2r = loc_robots1[:, 1].reshape(-1, 1) - loc_robots2[:, 1].reshape(1, -1)
        self._dis_robot2robot = np.sqrt(np.power(self._delta_x_r2r, 2) + np.power(self._delta_y_r2r, 2))

    def cal_dis_robot2workstand(self):
        # 计算所有机器人到所有工作站的距离 向量化 每来一帧调用一次
        # 距离表存在类变量中
        # 通过get_dis_robot2workstand(self, idx_robot, idx_workstand)调用
        loc_robots = self._robots.get_loc(-1)
        loc_workstands = self._workstands.get_loc(-1)
        self._delta_x_r2w = loc_robots[:, 0].reshape(-1, 1) - loc_workstands[:, 0].reshape(1, -1)
        self._delta_y_r2w = loc_robots[:, 1].reshape(-1, 1) - loc_workstands[:, 1].reshape(1, -1)
        self._dis_robot2workstand = np.sqrt(np.power(self._delta_x_r2w, 2) + np.power(self._delta_y_r2w, 2))

    def cal_dis_workstand2workstand(self):
        # 计算所有工作站两两之间的距离 向量化 只需要在初始化调用一次
        # 距离表存在类变量中
        # 通过get_dis_workstand2workstand(self, idx_workstand1, idx_workstand2)调用
        loc_workstands1 = self._workstands.get_loc(-1)
        loc_workstands2 = self._workstands.get_loc(-1)
        self._delta_x_w2w = loc_workstands1[:, 0].reshape(-1, 1) - loc_workstands2[:, 0].reshape(1, -1)
        self._delta_y_w2w = loc_workstands1[:, 1].reshape(-1, 1) - loc_workstands2[:, 1].reshape(1, -1)

        self._dis_workstand2workstand = np.sqrt(np.power(self._delta_x_w2w, 2) + np.power(self._delta_y_w2w, 2))

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

                dx_robot = self._delta_x_r2r[idx_robot, idx_other] / distance_robot  # 其他指向自己的方向余弦
                dy_robot = self._delta_y_r2r[idx_robot, idx_other] / distance_robot  # 其他指向自己的方向余弦

                theta_other = self._robots.get_status(feature_theta_r, idx_other)

                # 各向异性判断是否会产生潜在的碰撞事件
                dircos_other = np.array([math.cos(theta_other), math.sin(theta_other)])
                ang_robot = math.acos(np.dot(dircos_robot, np.array([-dx_robot, -dy_robot])))
                try:
                    ang_other = math.acos(np.dot(dircos_other, np.array([dx_robot, dy_robot])))
                except:
                    raise Exception([self._delta_x_r2r[idx_robot, idx_other],
                                     self._delta_y_r2r[idx_robot, idx_other],
                                     distance_robot])
                # 如果机器人之间的距离小于一定半径范围，则计算斥力
                if distance_robot < RADIUS and (ang_robot < math.pi * 0.2 or ang_other < math.pi * 0.2):
                    repulsive_force = 0.5 * ETA * ((1.0 / distance_robot) - (1.0 / RADIUS)) ** 2
                    repulsive_field[0] += repulsive_force * dx_robot
                    repulsive_field[1] += repulsive_force * dy_robot

        # 计算机器人到目标点的吸引力
        distance_r2w = self.get_dis_robot2workstand(idx_robot, idx_workstand)

        dx_r2w = -self._delta_x_r2w[idx_robot, idx_workstand] / distance_r2w  # 加负号后自己指向工作台
        dy_r2w = -self._delta_y_r2w[idx_robot, idx_workstand] / distance_r2w  # 加负号后自己指向工作台

        attractive_force = 0.5 * GAMMA * distance_r2w ** 2
        attractive_field[0] = attractive_force * dx_r2w
        attractive_field[1] = attractive_force * dy_r2w

        total_field = repulsive_field + attractive_field
        desired_angle = np.arctan2(total_field[1], total_field[0])
        return desired_angle

    def move2loc(self, idx_robot, speed):
        # 输入控制机器人编号 目标工作台编号 期望速度
        # 结合人工势场计算速度
        idx_target = self._robots.get_status(feature_target_r, idx_robot)

        desired_theta = self.calculate_potential_field(idx_robot, idx_target)

        # 比例控制 追踪目标方向
        # 计算相差方向 P
        now_theta = self._robots.get_status(feature_theta_r, idx_robot)
        now_ang_velo = self._robots.get_status(feature_ang_velo_r, idx_robot)
        delta_theta = desired_theta - now_theta
        delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi
        k = 10
        self._robots.rotate(idx_robot, delta_theta * k)
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
        if abs(delta_theta) > math.pi / 4:
            speed = delta_theta * k * 0.5
        self._robots.forward(idx_robot, speed)

    def get_time_rate(self, frame_sell: float) -> float:
        # 计算时间损失
        if frame_sell >= 9000:
            return 0.8
        sqrt_num = math.sqrt(1 - (1 - frame_sell / 9000) ** 2)
        return (1 - sqrt_num) * 0.2 + 0.8

    def compute_rest_time_to_finish_now_task(self,idx_robot):
        if int(self._robots.get_status(feature_status_r,idx_robot)) == 0:
            return 0
        if idx_robot not in self._idx_robox_2_task:
            return 0
        task = self._idx_robox_2_task[idx_robot]
        if task is None or not Task:
            return 0
        now_target = int(self._robots.get_status(feature_target_r,idx_robot))
        if now_target != task.get_sell_workstand_idx():
            return INFINITE
        rest_time = int(self.get_dis_robot2workstand(idx_robot,task.get_sell_workstand_idx())) * MOVE_SPEED
        return rest_time

    def get_speed_from_dis(self,dis):
        if dis >= 10:
            return 9999
        if dis >= 5:
            return 40
        if dis >= 3.5:
            return 16
        if dis >= 2:
            return 10
        if dis >= 1:
            return 7
        return  5

    def control(self):
        tasks = []
        robot_num_idle = 0
        for robot_idx in range(4):
            robot_status = int(self._robots.get_status(feature_status_r, robot_idx))
            if robot_status == 0:
                robot_num_idle+=1
        for product_workstand_idx in range(len(self._workstands)):
            product_workstand_type, product_time, _, product_status = self._workstands.get_workstand_status(
                product_workstand_idx)

            product_type = WORKSTAND_OUT[product_workstand_type]
            if product_type == None:
                # 不生成产品
                continue

            if product_status == 0:
                # 暂时没有产品
                if product_time == -1 or product_time > MAX_WAIT:
                    # 没有准备好原材料
                    continue

            if self._workstands.get_product_pro(product_workstand_idx) == 1:
                # 该产品已被预定
                continue

            for sell_workstand_idx in range(len(self._workstands)):
                """
                暂时未考虑是否优先将产品卖给同一台需要多种原料才能生产的工作站
                """
                sell_workstand_type, _, material, product_status = self._workstands.get_workstand_status(
                    sell_workstand_idx)
                if WORKSTAND_OUT[product_workstand_type] not in WORKSTAND_IN[sell_workstand_type]:
                    # 当前工作台不接收购买的产品
                    continue
                if (int(material) & ( 1 << product_type)) or \
                       (int(self._workstands.get_material_pro(sell_workstand_idx)) & (1 << product_type)):
                    # 当前工作台原材料格子里已经放入上面购买的原材料
                    # or 该原材料格子已被预定
                    continue

                for robot_idx in range(4):
                    robot_status = int(self._robots.get_status(feature_status_r, robot_idx))
                    # if robot_status != 0:
                    #     continue
                    rest_time = self.compute_rest_time_to_finish_now_task(robot_idx)
                    if robot_idx in self._idx_robox_2_task and product_workstand_idx \
                            == self._idx_robox_2_task[robot_idx].get_sell_workstand_idx():
                        rest_time = 0
                    dis_robot_product_workstand = self.get_dis_robot2workstand(robot_idx, product_workstand_idx)
                    dis_product_sell_workstand = self.get_dis_workstand2workstand(product_workstand_idx,
                                                                                  sell_workstand_idx)
                    price_buy = ITEMS_BUY[product_type]
                    price_sell = ITEMS_SELL[product_type]
                    frame_need = (rest_time  + dis_product_sell_workstand  * MOVE_SPEED+ \
                               (max(dis_robot_product_workstand * MOVE_SPEED , product_time))) * FRAME_MUL

                    if frame_need >= ALL_FRAME_COUNT - CURRENT_FRAME and rest_time != INFINITE:
                        continue

                    f = (rest_time  + dis_product_sell_workstand  * MOVE_SPEED+ \
                               (max(dis_robot_product_workstand * MOVE_SPEED , product_time)))
                    ratio = (price_sell  - price_buy) / f
                    if int(sell_workstand_type) in material_ok_count.get_key_of_min_count():
                        ratio *= 1.2
                    if sell_workstand_type != 9 and \
                            count_2(int(self._workstands.get_material_pro(sell_workstand_idx))) + count_2(int(material))==1:
                        ratio *=1.1
                    if sell_workstand_type != 9 and \
                            count_2(int(self._workstands.get_material_pro(sell_workstand_idx))) + count_2(int(material))>1:
                        ratio *= 1.4
                    if product_type >= 7:
                        ratio *= 100
                    elif product_type > 3:
                        ratio *=1.2
                    heapq.heappush(tasks, Task(ratio, robot_idx, product_workstand_idx, sell_workstand_idx))
        tasks = sorted(tasks,reverse=True)
        next_task_dict = dict()
        next_task_set = set()
        for task in tasks:
            robot_idx = task.get_robot_idx()
            if robot_idx in next_task_dict.keys() or task.get_key() in next_task_set:
                continue
            next_task_dict[robot_idx] = task
            next_task_set.add(task.get_key())
            if len(next_task_dict) >= 4:
                break
            product_workstand_idx = task.get_product_workstand_idx()
            sell_workstand_idx = task.get_sell_workstand_idx()

            robot_status = int(self._robots.get_status(feature_status_r, robot_idx))
            if robot_status != 0:
                continue

            product_workstand_type, _, _, _ = self._workstands.get_workstand_status(
                product_workstand_idx)

            if self._workstands.get_product_pro(product_workstand_idx) == 1:
                # 该产品格子已被预定
                continue
            product_type = WORKSTAND_OUT[product_workstand_type]
            materials = self._workstands.get_material_pro(sell_workstand_idx)
            if int(materials) & (1 << product_type):
                # 该原材料格子已被预定
                continue

            self._robots.set_status_item(feature_status_r, robot_idx, 1)
            self._robots.set_status_item(feature_target_r,robot_idx, product_workstand_idx)
            self._workstands.set_product_pro(product_workstand_idx, 1)
            self._workstands.set_material_pro(sell_workstand_idx, int(materials) | (1 << product_type))

            if product_type < 4:
                self._workstands.get_material_pro(sell_workstand_idx)
                sell_workstand_type = int(self._workstands.get_workstand_status(sell_workstand_idx)[0])
                material_ok_count.add(sell_workstand_type)

            self._idx_robox_2_task[robot_idx] = task

        idx_robot = 0
        while idx_robot < 4:
            robot_status = int(self._robots.get_status(feature_status_r, idx_robot))
            if robot_status == 0:
                idx_robot += 1
                continue
                # 【空闲】执行调度策略
                # self._robots.set_status_item(feature_target_r, 0)  # 这里以1为例 即准备卖给1
                # self._robots.set_status_item(feature_status_r, idx_robot, 1)  # 切换为 【购买途中】

                # 选择可购买的工作台
                # continue
            elif robot_status == 1:
                # 【购买途中】
                # 移动

                # 判断距离是否够近
                self.move2loc(idx_robot, self.get_speed_from_dis(\
                    self.get_dis_robot2workstand(idx_robot, self._robots.get_status(feature_target_r, idx_robot))))
                # 判定是否进入交互范围
                if self._robots.get_status(feature_workstand_id_r, idx_robot) == self._robots.get_status(
                        feature_target_r, idx_robot):
                    self.move2loc(idx_robot, 0)
                    self._robots.set_status_item(feature_status_r, idx_robot, 2)  # 切换为 【等待购买】
                    idx_robot += 1
                    continue



            elif robot_status == 2:
                # 【等待购买】
                # 如果在等待，提前转向
                product_workstand_idx = int(self._robots.get_status(feature_target_r, idx_robot))
                is_nearby_workstand = self._robots.get_status(feature_workstand_id_r,
                                                              idx_robot) == product_workstand_idx
                have_product = self._workstands.get_workstand_status(product_workstand_idx)[3]
                if is_nearby_workstand and have_product:  # 这里判定是否生产完成可以购买 不是真的1
                    # 可以购买
                    self._robots.buy(idx_robot)  # 防止购买失败
                    self._robots.set_status_item(feature_status_r, idx_robot, 3)  # 切换为 【出售途中】
                    self._workstands.set_product_pro(product_workstand_idx, 0)  # 取消预购
                    self._robots.set_status_item(feature_target_r, idx_robot, \
                                                 self._idx_robox_2_task[idx_robot].get_sell_workstand_idx())
                    idx_robot += 1
                    continue
                if not is_nearby_workstand:
                    self._robots.set_status_item(feature_status_r, idx_robot, 1)  # 切换为 【购买途中】
                    idx_robot += 1
                    continue
                if not have_product:
                    pass

            elif robot_status == 3:
                # 【出售途中】
                # 移动
                # 判断距离是否够近
                task = self._idx_robox_2_task[idx_robot]
                dis_from_pro = self.get_dis_robot2workstand(idx_robot, task.get_product_workstand_idx())
                dis_from_sell = self.get_dis_robot2workstand(idx_robot, task.get_sell_workstand_idx())
                self.move2loc(idx_robot, self.get_speed_from_dis(min(dis_from_pro,dis_from_sell))) # 减速

                # 判定是否进入交互范围
                if self._robots.get_status(feature_workstand_id_r, idx_robot) == self._robots.get_status(
                        feature_target_r, idx_robot):
                    self.move2loc(idx_robot,  0)  # 刹车
                    self._robots.set_status_item(feature_status_r, idx_robot, 4)  # 切换为 【等待出售】
                    idx_robot += 1
                    continue


            elif robot_status == 4:
                # 【等待出售】
                # 如果在等待，提前转向
                sell_workstand_idx = int(self._robots.get_status(feature_target_r, idx_robot))
                workstand_type ,_,materials,_= self._workstands.get_workstand_status(sell_workstand_idx)
                material_type = int(self._robots.get_status(feature_materials_r,idx_robot))
                is_nearby_workstand = self._robots.get_status(feature_workstand_id_r,
                                                              idx_robot) == sell_workstand_idx
                materials =int(materials)
                can_put_material = (int(materials >>  material_type) & 1) == 0
                if is_nearby_workstand and can_put_material:  # 能否放原材料
                    self._robots.sell(idx_robot)
                    self._robots.set_status_item(feature_status_r, idx_robot, 0)  # 切换为 【空闲】
                    self._workstands.set_material_pro(sell_workstand_idx,
                                                      clrbit(int(self._workstands.get_material_pro(sell_workstand_idx)),int(material_type)))  # 取消预购
                    self._robots.set_status_item(feature_target_r, idx_robot, -1)
                    self._idx_robox_2_task.pop(idx_robot)
                    self._robots.forward(idx_robot,6)
                    idx_robot += 1
                    continue
                if not is_nearby_workstand:
                    self._robots.set_status_item(feature_status_r, idx_robot, 3)  # 切换为 【售卖途中】
                    idx_robot += 1
                    continue
                if not can_put_material:
                    self._robots.set_status_item(feature_status_r,idx_robot,0)
                    self._robots.destroy(idx_robot)

            idx_robot += 1



def read_map(map_in: Map, robot_group_in: RobotGroup):
    num_robot = 0
    num_line = 0
    while True:
        line_read = input()
        if line_read == "OK":
            return
        else:
            for idx, point_str in enumerate(line_read):
                if point_str.isdigit() and 1 <= int(point_str) <= 9:
                    # 工作台
                    x = num_line * 0.5 + 0.25
                    y = idx * 0.5 + 0.25
                    map_in.add_workstand(int(point_str), x, y)
                elif point_str == 'A':
                    # 机器人
                    x = num_line * 0.5 + 0.25
                    y = idx * 0.5 + 0.25
                    robot_group_in.add_init_location(num_robot, x, y)
                    num_robot += 1
        num_line += 1


def init_ITEMS_NEED(workstands: Map):
    for idx in range(len(workstands)):

        typeID = int(workstands.get_workstand_status(idx)[0])
        for itemID in WORKSTAND_IN[typeID]:
            ITEMS_NEED[itemID].append(idx)


def get_info(map_in, robot_group):
    line_read = input()
    frame_id, money = line_read.split(' ')

    line_read = input()
    num_robot = 4
    num_platform = int(line_read)

    # 更新工作台
    for id_platform in range(num_platform):
        line_read = input()
        map_in.update_platform(id_platform, line_read)

    # 更新机器人
    for id_robot in range(num_robot):
        line_read = input()
        robot_group.update_robot(id_robot, line_read)

    line_read = input()
    if line_read == 'OK':
        return frame_id, money
    else:
        # 到这就出错了
        pass


def finish():
    # logging.info('OK\n')
    sys.stdout.write('OK\n')
    sys.stdout.flush()


if __name__ == '__main__':
    # logging.basicConfig(filename='log/log.log', level=# logging.DEBUG)
    # logging.info('===================')
    robot_group_obj = RobotGroup()
    map_obj = Map()
    # time.sleep(20)
    read_map(map_obj, robot_group_obj)
    init_ITEMS_NEED(map_obj)

    controller = Controller(robot_group_obj, map_obj)
    # time.sleep(20)
    # 只需计算一次
    controller.cal_dis_workstand2workstand()
    finish()
    while True:
        frame_id, money = get_info(map_obj, robot_group_obj)
        CURRENT_FRAME = int(frame_id)
        controller.cal_dis_robot2workstand()
        controller.cal_dis_robot2robot()
        print(frame_id)
        controller.control()
        finish()
