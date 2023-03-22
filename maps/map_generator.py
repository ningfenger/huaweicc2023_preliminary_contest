import random
import time

random.seed(time.time())
def gen_map():
    robot_num = 4
    walkstand_num = random.randint(0, 40)
    locs = set()
    def gen_loc():
        while 1:
            x = random.randint(0, 99)
            y = random.randint(0, 99)
            if (x, y) in locs:
                continue
            for a in range(max(x-7,0), min(x+7,100)):
                for b in range(max(y-7,0), min(y+7,100)):
                    locs.add((a, b))
            return x,y
    new_map = [['.'] * 100 for _ in range(100)]
    for _ in range(robot_num):
        x, y = gen_loc()
        new_map[x][y] = 'A'
    # 随机生成一个 8 或 9
    x, y = gen_loc()
    if random.randint(0,1) == 1:
        new_map[x][y] = '9'
    else:
        new_map[x][y] = '8'
    # 随机生成1-7
    for i in range(1,8):
        x, y = gen_loc()
        new_map[x][y] = str(i)
    # 随机生成剩余部分
    for _ in range(walkstand_num):
        x, y = gen_loc()
        walkstand_type = random.randint(1,8)
        new_map[x][y] = str(walkstand_type)
    # for line in new_map:
    #     print(''.join(line))
    return [''.join(line)+'\n' for line in new_map]
if __name__ == "__main__":
    for i in range(200,201):
        file = open(f'maps/{i}.txt', 'w')
        new_map = gen_map()
        file.writelines(new_map)
