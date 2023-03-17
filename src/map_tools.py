# coding=utf-8
def robot_move(robot, workstand):
    '''
    :param robot 机器人对象
    :param target 目标工作台对象
    :return forward 机器人线速度
    :return rotate 机器人角速度
    :return nextloc 下一帧预计会处于的位置，避障用，如果提前一帧预判不能避障考虑修改为预计几帧后的位置
    '''
    pass


def move_time(workstand1, workstand2):
    # 预估两个工作台间的时间
    pass

def will_clash(robot1, robot2):
    # 预判两个机器人是否会碰撞
    pass

def will_clash(loc1, loc2):
    # 判断两个坐标是否会碰撞
    pass

def get_dismap(workstands):
    # 计算出全部工作台两两间的距离
    pass