# -*-coding:utf-8-*-
# @Time       : 2023/3/22 14:38
# @Author     : Feng Rui
# @Site       : 
# @File       : load_pkl.py
# @Software   : PyCharm
# @Description:
import pickle

with open('record.pkl', 'rb') as f:
    data = pickle.load(f)

pass