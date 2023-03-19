#!/bin/bash
# coding=utf-8

import os

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