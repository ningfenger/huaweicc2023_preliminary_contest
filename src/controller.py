# coding=utf-8
from robot import *
from workstand import *
import numpy as np
import copy
import math
import logging

# 环境常量
MATCH_FRAME = 3*60*50  # 总帧数
DISMAP = None  # 初始化时更新，记录任意两个工作台间的测算距离/帧数
ITEMS_BUY = [0, 3000, 4400, 5800, 15400, 17200, 19200, 76000]  # 每个物品的购买价
ITEMS_SELL = [0, 6000, 7600, 9200, 22500, 25000, 27500, 105000]
ITEMS_NEED = [[] for _ in range(8)]  # 记录收购每个商品的工作台编号
WORKSTAND_IN = {1: [], 2: [], 3: [], 4: [1, 2], 5: [1, 3],
                6: [2, 3], 7: [4, 5, 6], 8: [7], 9: list(range(1, 8))}
WORKSTAND_OUT = {i: i for i in range(1, 8)}
WORKSTAND_OUT[8] = None
WORKSTAND_OUT[9] = None

# 测试
DEBUG = False


class Controller:
    # 控制参数
    DIS_1 = 1.3
    VELO_1 = 1.1
    MOVE_SPEED = 1 / 4 * 50  # 估算移动时间
    MAX_WAIT = 3 * 50  # 最大等待时间
    SELL_WEIGHT = 1.2  # 优先卖给格子被部分占用的
    # 人工势场常数
    ETA = 300  # 调整斥力大小的常数
    GAMMA = 10  # 调整吸引力大小的常数
    RADIUS = 3  # 定义斥力半径范围
    BUY_WEIGHT = [1]*4+[1]*3+[1]  # 购买优先级，优先购买高级商品
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
        if DEBUG:
            logging.basicConfig(filename='debug.log', level=logging.DEBUG)

    def init_ITEMS_NEED(self):
        workstands = self._workstands
        for idx in range(len(workstands)):
            typeID = int(workstands.get_workstand_status(idx)[0])
            for itemID in WORKSTAND_IN[typeID]:
                ITEMS_NEED[itemID].append(idx)

    def set_control_parameters(self, dis_1:float, velo_1:float, move_speed:float, max_wait:int, sell_weight:float, eta:float, gamma:float, radius:float):
        self.DIS_1 = dis_1
        self.VELO_1 = velo_1
        self.MOVE_SPEED = move_speed  # 估算移动时间
        self.MAX_WAIT = max_wait  # 最大等待时间
        self.SELL_WEIGHT = sell_weight  # 优先卖给格子被部分占用的
        # 人工势场常数
        self.ETA = eta  # 调整斥力大小的常数
        self.GAMMA = gamma  # 调整吸引力大小的常数
        self.RADIUS = radius  # 定义斥力半径范围


    def cal_dis_robot2robot(self):
        # 计算所有机器人两两之间的距离 向量化 每来一帧调用一次
        # 距离表存在类变量中
        # 通过get_dis_robot2robot(self, idx_robot, idx_workstand)调用
        loc_robots1 = self._robots.get_loc(-1)
        loc_robots2 = self._robots.get_loc(-1)
        self._delta_x_r2r = loc_robots1[:,
                                        0].reshape(-1, 1) - loc_robots2[:, 0].reshape(1, -1)
        self._delta_y_r2r = loc_robots1[:,
                                        1].reshape(-1, 1) - loc_robots2[:, 1].reshape(1, -1)
        self._dis_robot2robot = np.sqrt(
            np.power(self._delta_x_r2r, 2) + np.power(self._delta_y_r2r, 2))

    def cal_dis_robot2workstand(self):
        # 计算所有机器人到所有工作站的距离 向量化 每来一帧调用一次
        # 距离表存在类变量中
        # 通过get_dis_robot2workstand(self, idx_robot, idx_workstand)调用
        loc_robots = self._robots.get_loc(-1)
        loc_workstands = self._workstands.get_loc(-1)
        self._delta_x_r2w = loc_robots[:, 0].reshape(
            -1, 1) - loc_workstands[:, 0].reshape(1, -1)
        self._delta_y_r2w = loc_robots[:, 1].reshape(
            -1, 1) - loc_workstands[:, 1].reshape(1, -1)
        self._dis_robot2workstand = np.sqrt(
            np.power(self._delta_x_r2w, 2) + np.power(self._delta_y_r2w, 2))

    def cal_dis_workstand2workstand(self):
        # 计算所有工作站两两之间的距离 向量化 只需要在初始化调用一次
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

                # 其他指向自己的方向余弦
                dx_robot = self._delta_x_r2r[idx_robot,
                                             idx_other] / distance_robot
                # 其他指向自己的方向余弦
                dy_robot = self._delta_y_r2r[idx_robot,
                                             idx_other] / distance_robot

                theta_other = self._robots.get_status(
                    feature_theta_r, idx_other)

                # 各向异性判断是否会产生潜在的碰撞事件
                dircos_other = np.array(
                    [math.cos(theta_other), math.sin(theta_other)])
                ang_robot = math.acos(
                    np.dot(dircos_robot, np.array([-dx_robot, -dy_robot])))
                try:
                    ang_other = math.acos(
                        np.dot(dircos_other, np.array([dx_robot, dy_robot])))
                except:
                    raise Exception([self._delta_x_r2r[idx_robot, idx_other],
                                     self._delta_y_r2r[idx_robot, idx_other],
                                     distance_robot])
                # 如果机器人之间的距离小于一定半径范围，则计算斥力
                if distance_robot < self.RADIUS and (ang_robot < math.pi * 0.2 or ang_other < math.pi * 0.2):
                    repulsive_force = 0.5 * self.ETA * \
                        ((1.0 / distance_robot) - (1.0 / self.RADIUS)) ** 2
                    repulsive_field[0] += repulsive_force * dx_robot
                    repulsive_field[1] += repulsive_force * dy_robot

        # 计算机器人到目标点的吸引力
        distance_r2w = self.get_dis_robot2workstand(idx_robot, idx_workstand)

        dx_r2w = -self._delta_x_r2w[idx_robot,
                                    idx_workstand] / distance_r2w  # 加负号后自己指向工作台
        dy_r2w = -self._delta_y_r2w[idx_robot,
                                    idx_workstand] / distance_r2w  # 加负号后自己指向工作台

        attractive_force = 0.5 * self.GAMMA * distance_r2w ** 2
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

    def set_pro_buy(self, idx_robot: int, buy=True):
        # 设置预定
        target_walkstand, _ = self._robots.robots_plan[idx_robot]
        # 预定工作台
        if buy:
            workstand_type = int(
                self._workstands.get_workstand_status(target_walkstand)[0])
            if not workstand_type in [1, 2, 3]:  # 123不锁
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
        next_walkstand_type= int(
            self._workstands.get_workstand_status(next_walkstand)[0])
        if next_walkstand_type in [8,9]:
            return
        if sell:
            self._workstands.set_material_pro(
                next_walkstand, material_pro + (1 << workstand_types))
        else:
            self._workstands.set_material_pro(
                next_walkstand, material_pro - (1 << workstand_types))
            


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
                if WORKSTAND_OUT[sell_type] and 1 << workstand_type & sell_material:
                    continue
                    # if sell_product_time == -1:  # 剩余生产时间为0，说明生产阻塞
                    #     continue
                    # else:
                    #     frame_wait_sell = sell_product_time
                frame_move_to_sell = self.get_dis_workstand2workstand(
                    idx_workstand, idx_worksand_to_sell) * self.MOVE_SPEED
                frame_buy = max(frame_move_to_buy,
                                frame_wait_buy)  # 购买时间
                # frame_sell = max(frame_move_to_sell,
                #                  frame_wait_sell - frame_buy)  # 出售时间
                total_frame = frame_buy + frame_move_to_sell  # 总时间
                if total_frame + frame_id > MATCH_FRAME:  # 完成这套动作就超时了
                    continue
                time_rate = self.get_time_rate(
                    frame_move_to_sell)  # 时间损耗
                sell_weight = self.SELL_WEIGHT if sell_material else 1
                radio = (
                    ITEMS_SELL[workstand_type] * time_rate - ITEMS_BUY[
                        workstand_type]) / total_frame*sell_weight*buy_weight
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
        idx_robot = 0
        sell_out_list = [] # 等待处理预售的机器人列表
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
                                                self._robots.get_status(feature_target_r, idx_robot)) < self.DIS_1:
                    self.move2loc(idx_robot, self.VELO_1)
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
                        self.set_pro_buy(idx_robot, False)
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
                                                self._robots.get_status(feature_target_r, idx_robot)) < self.DIS_1:
                    self.move2loc(idx_robot, self.VELO_1)
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
