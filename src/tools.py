import math
import numpy as np

def sign_pow(num_in, n):
    if num_in < 0:
        return -abs(num_in) ** n
    else:
        return abs(num_in) ** n



def get_dx_dy_d(a, b):
    dx = b[:, 0].reshape(1, -1) - a[:, 0].reshape(-1, 1)
    dy = b[:, 1].reshape(1, -1) - a[:, 1].reshape(-1, 1)
    d = np.sqrt(np.power(dx, 2) + np.power(dy, 2))
    return dx, dy, d

def will_collide(x1, y1, vx1, vy1, x2, y2, vx2, vy2, t_max, r = 0.53):
    # 计算机器人之间的初始距离
    dist = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    # 计算相对速度
    rel_vx = vx1 - vx2
    rel_vy = vy1 - vy2
    # 如果机器人的相对速度为零，则它们永远不会相遇
    if rel_vx == 0 and rel_vy == 0:
        return (False, None, None, None, None)
    # 计算参数方程
    a = rel_vx**2 + rel_vy**2
    b = 2 * ((x1 - x2) * rel_vx + (y1 - y2) * rel_vy)
    c = (x1 - x2)**2 + (y1 - y2)**2 - 4 * r**2
    delta = b**2 - 4 * a * c
    # 如果delta小于零，则机器人之间不会相遇
    if delta < 0:
        return (False, None, None, None, None)
    else:
        t1 = (-b + math.sqrt(delta)) / (2 * a)
        t2 = (-b - math.sqrt(delta)) / (2 * a)
        t = min(t1, t2)
        # 如果时间是负数或者超出了预测的时间范围，则机器人之间不会相遇
        if t < 0 or t > t_max:
            return (False, None, None, None, None)
        # 计算碰撞点的位置
        collision_x = (x1 + vx1 * t + x2 + vx2 * t) / 2
        collision_y = (y1 + vy1 * t + y2 + vy2 * t) / 2
        # 计算碰撞点距离各自的长度
        distance1 = math.sqrt((collision_x - x1)**2 + (collision_y - y1)**2) - r
        distance2 = math.sqrt((collision_x - x2)**2 + (collision_y - y2)**2) - r
        return (True, collision_x, collision_y, distance1, distance2)



def will_collide2(x_robot, y_robot, vx_robot, vy_robot, x_other, y_other, vx_other, vy_other, t):
    # 计算机器人和另一个物体之间的相对速度
    vx_rel = vx_robot - vx_other
    vy_rel = vy_robot - vy_other

    # 计算机器人和另一个物体之间的相对位置
    x_rel = x_robot - x_other
    y_rel = y_robot - y_other

    # 如果两个物体在x和y方向上的相对速度和相对位置都相反，则它们会碰撞
    if vx_rel * x_rel + vy_rel * y_rel < 0:
        # 计算碰撞时间
        t_col = -(vx_rel * x_rel + vy_rel * y_rel) / (vx_rel ** 2 + vy_rel ** 2)

        # 计算碰撞点的坐标
        x_col = x_robot + vx_robot * t_col
        y_col = y_robot + vy_robot * t_col

        # 计算碰撞点距离各自的长度
        dist_robot = ((x_col - x_robot) ** 2 + (y_col - y_robot) ** 2) ** 0.5
        dist_other = ((x_col - x_other) ** 2 + (y_col - y_other) ** 2) ** 0.5

        # 判断碰撞时间是否在t时间内
        if t_col < t:
            return True, x_col, y_col, dist_robot, dist_other

    return False, None, None, None, None
