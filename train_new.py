#!/bin/bash
# coding=utf-8

import os
import random
from sys import platform, argv
import time

Robot = 'Robot.exe'
if platform == 'linux':
    Robot = './Robot'

# 暴力找到最优解，获得一个数据集
def find_best(map_num):
    best_value = 0
    best_param = []
    print(map_num)
    for move_speed in range(2,6):
        for max_wait in range(2,8):
            for sell_weight in range(6):
                for sell_debuff in range(5,11):
                    MOVE_SPEED = 1 / move_speed * 50
                    MAX_WAIT = max_wait * 50
                    SELL_WEIGHT = 1+sell_weight/10
                    SELL_DEBUFF = sell_debuff/10
                    cmd = f'{Robot} "python src/main.py --move_speed {MOVE_SPEED} --max_wait {MAX_WAIT} --sell_weight {SELL_WEIGHT} --sell_debuff {SELL_DEBUFF}" -f -m maps/{map_num}.txt'
                    try:
                        res = os.popen(cmd).readlines()[-1]
                        score = eval(res)['score']
                        param = [MOVE_SPEED, MAX_WAIT, SELL_WEIGHT, SELL_DEBUFF]
                        print(f"{param}:{score}")
                        if score > best_value:
                            best_value = score
                            best_param = param
                    except:
                        return None
    return best_param


def gen_datebase(map_range_start, map_range_end):
    for map_id in range(map_range_start, map_range_end):
        bast_param = find_best(map_id)
        if not bast_param:
            continue
        bast_param = map(str, bast_param)
        time_tuple = time.localtime(time.time())
        map_name = f'maps/{map_id}.txt'
        file_name = f'database/{time_tuple[0]}-{time_tuple[1]}-{time_tuple[2]}-{time_tuple[3]}_{time_tuple[4]}_{time_tuple[5]}_map{map_id}.txt'
        data_file = open(file_name,'w')
        data_file.write(','.join(bast_param)+'\n')
        map_file = open(map_name, 'r')
        lines = map_file.readlines()
        data_file.writelines(lines)
        data_file.close()
        map_file.close()


if __name__ == '__main__':
    gen_datebase(int(argv[1]), int(argv[2]))
