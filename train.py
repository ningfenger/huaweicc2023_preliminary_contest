# coding=utf-8

import os
import random
from sys import platform

Robot = 'Robot.exe'
if platform == 'linux':
    Robot = './Robot'

# 转化归一化的值


def get_final_value(x1, x2, x3, x4, x5, x6, x7, x8, x9):
    DIS_1 = 0.3+x1  # 开始刹车的距离 0.3~1.3
    VELO_1 = 0.1+x2  # 刹车时的速度 0.1~1.1
    MOVE_SPEED = 1 / (2+3*x3) * 50  # 估算移动时间 2~5
    MAX_WAIT = (2+8*x4) * 50  # 最大等待时间 2~10
    SELL_WEIGHT = 1+x5  # 优先卖给格子被部分占用的 1~2
    SELL_DEBUFF = x6 # 0~1.0
    # 人工势场常数
    ETA = 100+400*x7  # 调整斥力大小的常数 100~500
    GAMMA = 5+15*x8  # 调整吸引力大小的常数 5~20
    RADIUS = 2+8*x9  # 定义斥力半径范围 2-10
    return DIS_1, VELO_1, MOVE_SPEED, MAX_WAIT, SELL_WEIGHT, ETA, GAMMA, RADIUS
# 定义目标函数


def target_function(x1, x2, x3, x4, x5, x6, x7, x8, x9):
    # 在这里写入你的目标函数代码
    maps = list(range(1, 5))
    total_score = 0
    DIS_1, VELO_1, MOVE_SPEED, MAX_WAIT, SELL_WEIGHT,SELL_DEBUFF, ETA, GAMMA, RADIUS = get_final_value(
        x1, x2, x3, x4, x5, x6, x7, x8, x9)
    print(f'参数: --dis_1 {DIS_1} --velo_1 {VELO_1} --move_speed {MOVE_SPEED} --max_wait {MAX_WAIT} \
        --sell_weight {SELL_WEIGHT} --sell_debuff {SELL_DEBUFF} --eta {ETA} --gamma {GAMMA} --radius {RADIUS}')
    for map in maps:
        # 控制参数
        cmd = f'{Robot} "python src/main.py --dis_1 {DIS_1} --velo_1 {VELO_1} --move_speed {MOVE_SPEED} --max_wait {MAX_WAIT} \
        --sell_weight {SELL_WEIGHT} --eta {ETA} --gamma {GAMMA} --radius {RADIUS} " -f -m maps/{map}.txt'
        res = os.popen(cmd).readlines()[-1]
        score = eval(res)['score']
        total_score += score
        print(f'地图{map}得分为: {score}')
    print(f'总得分为: {total_score}')
    return total_score/10**6


def train(num_iterations=100):
    # 定义可行解的范围（最小值和最大值）
    boundaries = [(0, 1.0)]*8
    # 定义初始参数
    params = [random.uniform(boundaries[i][0], boundaries[i][1])
              for i in range(len(boundaries))]
    # 开始迭代
    bast_value = 0
    bast_param = []
    delta = 0.05
    learning_rate = delta*0.5
    print(f'num_iterations:{num_iterations}, learning_rate:{learning_rate}')
    for i in range(num_iterations):
        print(F"\n\n第{i+1}轮训练"+'.'*100)
        # 计算梯度
        grad = []
        old_value = target_function(*params)
        for j in range(len(params)):
            new_params = params.copy()
            new_params[j] += delta
            new_value = target_function(*new_params)
            if new_value > bast_value:
                bast_value = new_value
                bast_param = new_params.copy()
            derivative = (new_value - old_value) / delta
            grad.append(derivative)

        # 更新参数
        for j in range(len(params)):
            params[j] += learning_rate * grad[j]
            # 确保参数在可行解范围内
            params[j] = max(params[j], boundaries[j][0])
            params[j] = min(params[j], boundaries[j][1])
        print("截至目前最优解为：", get_final_value(*bast_param))
        print("截至目前最优值为：", bast_value)
    print("最优解为：", get_final_value(*bast_param))
    print("最优值为：", bast_value)
    # 定义初始参数
    # initial_params = [random.uniform(
    # boundaries[i][0], boundaries[i][1]) for i in range(len(boundaries))]

    # current_value = target_function(*initial_params)

    # 尝试在每个方向上增加/减少参数，并找到最好的参数组合
    # best_value = current_value
    # 开始迭代
    # for i in range(num_iterations):
    #     print(F"\n\n第{i+1}轮训练.....................................")
    #     # 计算当前参数下的函数值
    #     best_params = initial_params.copy()
    #     for j in range(len(initial_params)):
    #         # 尝试增加/减少参数值
    #         new_params = initial_params.copy()
    #         new_params[j] += step_size
    #         if new_params[j] > boundaries[j][1]:
    #             new_params[j] = boundaries[j][1]
    #         new_value = target_function(*new_params)
    #         if new_value > best_value:
    #             best_value = new_value
    #             best_params = new_params
    #         else:
    #             # 尝试减少参数值
    #             new_params = initial_params.copy()
    #             new_params[j] -= step_size
    #             if new_params[j] < boundaries[j][0]:
    #                 new_params[j] = boundaries[j][0]
    #             new_value = target_function(*new_params)
    #             if new_value > best_value:
    #                 best_value = new_value
    #                 best_params = new_params

    # 更新参数
    # initial_params = best_params

    # print("最优解为：", initial_params)
    # print("最优值为：", target_function(*initial_params))


if __name__ == '__main__':
    train()
