@echo off
set PYTHONPATH=%cd%
set PYTHONIOENCODING=utf-8
set PYCHARM_DEBUG=True
set PYCHARM_DEBUG_PORT=5678
start /B "" robot_gui.exe "python Demo/main.py" -m maps\2.txt -d
