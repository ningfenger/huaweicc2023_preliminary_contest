# coding=utf-8
from typing import Optional, List, Tuple


class Workstand:
    DISMAP = None  # 初始化时更新，记录任意两个工作台间的测算距离/帧数
    ITEMS_BUY = [0, 3000, 4400, 5800, 15400, 17200, 19200, 76000]  # 每个物品的购买价
    ITEMS_SELL = [0, 6000, 7600, 9200, 22500, 25000, 27500, 105000]
    ITEMS_NEED = 0+[[] for _ in range(7)]  # 记录收购每个商品的工作台编号

    def __init__(self, ID: int, type: int, loc: Tuple):
        self.ID = ID  # 编号
        self.tpye = type  # 类型
        self.loc = loc  # 位置
        self.product_time = 0  # 剩余生产时间
        self.material = 0  # 原材料格状态
        self.product_status = 0  # 产品格状态
        self.product = 0  # 具体生产什么物品
        self.material_pro = 0  # 原料格预定状态, 防止有多个机器人将其作为出售目标 get set
        self.product_pro = 0  # 防止有多个机器人将其作为购买目标 get set

    def pro_sell(self, ateial_ID):
        # 预售接口 如果是89特殊处理
        pass

    def pro_buy(self):
        # 预购接口, 如果是123特殊处理
        pass

    def update(self, str):
        # 根据判题器传来的状态修订本机状态
        pass
