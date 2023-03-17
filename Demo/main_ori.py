#!/bin/bash
import sys
import logging


def read_util_ok():
    while input() != "OK":
        pass


def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()


if __name__ == '__main__':
    logging.basicConfig(filename='LOG2.log', encoding='utf-8', level=logging.DEBUG)
    read_util_ok()
    finish()
    while True:
        line = sys.stdin.readline()
        logging.info('line:' + line)
        if not line:
            break
        parts = line.split(' ')
        frame_id = int(parts[0])
        read_util_ok()

        sys.stdout.write('%d\n' % frame_id)
        line_speed, angle_speed = 3, 1.5
        for robot_id in range(4):
            sys.stdout.write('forward %d %d\n' % (robot_id, line_speed))
            sys.stdout.write('rotate %d %f\n' % (robot_id, angle_speed))
        finish()
