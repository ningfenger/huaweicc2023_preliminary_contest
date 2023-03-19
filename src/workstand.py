# coding=utf-8
import numpy as np
import copy

# _w结尾 工作台特征
feature_num_type_w = 0
feature_x_w = 1
feature_y_w = 2
feature_waiting_time_w = 3
feature_raw_state_w = 4
feature_product_state_w = 5
feature_material_pro_w = 6
feature_product_pro_w = 7

ITEMS_BUY = [0, 3000, 4400, 5800, 15400, 17200, 19200, 76000]  # 每个物品的购买价
ITEMS_SELL = [0, 6000, 7600, 9200, 22500, 25000, 27500, 105000]
WORKSTAND_IN = {1: [], 2: [], 3: [], 4: [1, 2], 5: [1, 3],
                6: [2, 3], 7: [4, 5, 6], 8: [7], 9: list(range(1, 8))}
WORKSTAND_OUT = {i: i for i in range(1, 8)}
WORKSTAND_OUT[8] = None
WORKSTAND_OUT[9] = None


class Map:
    def __init__(self):
        # 工作台信息 array
        self._workstand = np.array([]).reshape(0, 8)
        self.count = 0
        self.count_cell = 0
        # 下面的变量初始化后提供给控制类决策分析
        self.product_workstand_dict = {}
        self.receive_cell_dict = {}

    def add_workstand(self, num_type, x, y):
        # 第一次读地图添加工作台
        # 类型 x y 剩余生产时间 原材料格状态 产品格状态 预售接口 预购接口
        new_info = np.array(
            [num_type, x, y, 0.0, 0.0, 0.0] + [0] * 2).reshape(1, 8)
        self._workstand = np.concatenate([self._workstand, new_info], axis=0)

        item_product = WORKSTAND_OUT[num_type]
        if item_product is not None:
            buy_price = ITEMS_BUY[item_product]
            self.product_workstand_dict[self.count] = tuple([x, y, buy_price, item_product])  # 记录这个工作台产物的购买价格

        else:
            # 此工作台不生产任何物品
            buy_price = np.inf
            self.product_workstand_dict[self.count] = tuple([x, y, buy_price, np.nan])  # 记录这个工作台产物的购买价格

        item_receive_set = WORKSTAND_IN[num_type]
        if len(item_receive_set) > 0:
            for item_receive in item_receive_set:
                # 遍历此工作台接受的物品
                sell_price = ITEMS_SELL[item_receive]
                self.receive_cell_dict[self.count_cell] = tuple([x, y, sell_price, item_receive])
                self.count_cell += 1
        else:
            # 此工作台不接收任何物品 不做任何操作
            pass

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
        self._workstand[idx_workstand, -1] = value

    def get_material_pro(self, idx_workstand):
        # 原料格预定状态, 防止有多个机器人将其作为出售目标
        return copy.deepcopy(self._workstand[idx_workstand, -2])

    def get_product_pro(self, idx_workstand):
        # 防止有多个机器人将其作为购买目标
        return copy.deepcopy(self._workstand[idx_workstand, -1])

    def get_workstand_status(self, idx_workstand):
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
