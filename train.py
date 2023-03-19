#!/bin/bash
# coding=utf-8

import os
import random

# 定义目标函数
def target_function(x1, x2, x3, x4, x5, x6, x7, x8):
    # 在这里写入你的目标函数代码
    return result

# 定义可行解的范围（最小值和最大值）
boundaries = [(min_x1, max_x1), (min_x2, max_x2), (min_x3, max_x3), 
              (min_x4, max_x4), (min_x5, max_x5), (min_x6, max_x6), 
              (min_x7, max_x7), (min_x8, max_x8)]

# 定义初始参数
initial_params = [random.uniform(boundaries[i][0], boundaries[i][1]) for i in range(len(boundaries))]

# 定义步长
step_size = 0.1

# 定义迭代次数
num_iterations = 1000

# 开始迭代
for i in range(num_iterations):
    # 计算当前参数下的函数值
    current_value = target_function(*initial_params)
    
    # 尝试在每个方向上增加/减少参数，并找到最好的参数组合
    best_value = current_value
    best_params = initial_params.copy()
    for j in range(len(initial_params)):
        # 尝试增加/减少参数值
        new_params = initial_params.copy()
        new_params[j] += step_size
        if new_params[j] > boundaries[j][1]:
            new_params[j] = boundaries[j][1]
        new_value = target_function(*new_params)
        if new_value > best_value:
            best_value = new_value
            best_params = new_params
        else:
            # 尝试减少参数值
            new_params = initial_params.copy()
            new_params[j] -= step_size
            if new_params[j] < boundaries[j][0]:
                new_params[j] = boundaries[j][0]
            new_value = target_function(*new_params)
            if new_value > best_value:
                best_value = new_value
                best_params = new_params
                
    # 更新参数
    initial_params = best_params
    
print("最优解为：", initial_params)
print("最优值为：", target_function(*initial_params))
    

if __name__ == '__main__':
    maps = ['1', '2', '3', '4']
    total_score = 0
    for map in maps:
        # 控制参数
        DIS_1 = 0.4
        VELO_1 = 0.1
        MOVE_SPEED = 1 / 4 * 50  # 估算移动时间
        MAX_WAIT = 3 * 50  # 最大等待时间
        SELL_WEIGHT = 1.2  # 优先卖给格子被部分占用的
        # 人工势场常数
        ETA = 300  # 调整斥力大小的常数
        GAMMA = 10  # 调整吸引力大小的常数
        RADIUS = 4  # 定义斥力半径范围
        cmd = f'Robot.exe "python src\main.py --dis_1 {DIS_1} --velo_1 {VELO_1} --move_speed {MOVE_SPEED} --max_wait {MAX_WAIT} \
        --sell_weight {SELL_WEIGHT} --eta {ETA} --gamma {GAMMA} --radius {RADIUS} " -f -m maps\{map}.txt'
        print(cmd)
        res = os.popen(cmd).readlines()[-1]
        score = eval(res)['score']
        total_score += score
        print(f'地图{map}得分为: {score}')
    print('总得分:', total_score)