# -*-coding:utf-8-*-
# @Time       : 2023/3/22 18:36
# @Author     : Feng Rui
# @Site       : 
# @File       : fit.py
# @Software   : PyCharm
# @Description:

from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
import pickle
import numpy as np

with open('record.pkl', 'rb') as f:
    data = pickle.load(f)


num = len(data)
data = (np.absolute(np.array(data))).T

x= np.array([data[0],data[1]])
y = data[2]

def func(x,a,b):
    y = a*x[0] + b*x[1]
    return y

popt,pcov=curve_fit(func,x,y)
# popt[0]是参数a，popt[1]是参数b
print(popt)

pred =np.array(func([data[0], data[1]],*popt))

xlabel = np.arange(0,len(pred))
plt.plot(xlabel,pred)
plt.plot(xlabel,y)
plt.show()