import os
from collections import Counter
import statistics
import numpy as np
import re
import  matplotlib.pyplot as plt


data_pth = "C:/Users/RexJian/Desktop/nine_mul_six.log"
f = open(data_pth , 'r')
x_ls = []
y_ls = []
z_ls = []
x_diff = []
y_diff = []
z_diff = []
combine_diff = []
x_axis1 = []
lines = f.readlines()
for i , line in enumerate(lines):
    if i != 0:
        x = line[line.index(":") + 1:line.index(",")]
        y = line[line.index(",")+3:line.rindex(",")]
        z = line[line.rindex(":")+1 : len(line)-1]
        x_ls.append(float(x))
        y_ls.append(float(y))
        z_ls.append(float(z))
        x_axis1.append(i)
pre_val_x = 0
pre_val_y = 0
pre_val_z = 0
x_axis2 =[]
for i,(x,y,z) in enumerate(zip(x_ls,y_ls,z_ls)):
    if i != 0:
        x_diff.append(abs(float(x) - pre_val_x))
        y_diff.append(abs(float(y) - pre_val_y))
        z_diff.append(abs(float(z) - pre_val_z))
        combine_diff.append(abs(float(x) - pre_val_x) + abs(float(y) - pre_val_y) + abs(float(z) - pre_val_z))
        x_axis2.append(i)
    pre_val_x = float(x)
    pre_val_y = float(y)
    pre_val_z = float(z)
plt.figure(1)
plt.plot(x_axis2, x_diff, marker =".")
plt.title('X_DIFF')
plt.figure(2)
plt.plot(x_axis2, y_diff, marker =".")
plt.title('Y_DIFF')
plt.figure(3)
plt.plot(x_axis2, z_diff, marker =".")
plt.title('Z_DIFF')
plt.figure(4)
plt.plot(x_axis1, x_ls, marker =".")
plt.title('X')
plt.figure(5)
plt.plot(x_axis1, y_ls, marker =".")
plt.title('Y')
plt.figure(6)
plt.plot(x_axis1, z_ls, marker =".")
plt.title('Z')
# plt.figure(4)
# plt.plot(x_axis2, combine_diff, marker =".")
# plt.title('Combine')
plt.show()
# Analyze length

# def plt_figure(fig_num,y,title):
#     plt.figure(fig_num)
#     plt.plot(classes,y,marker=".")
#     for kind,val in zip(classes,y):
#         plt.text(kind,val,val,ha='right',va='top',fontsize=11)
#     plt.title(title)
# data_pth="../data/yagiyang"
# classes=["zero","one","two","three","four","five","six","seven","eight","nine"]
# median_list=[]
# mean_list=[]
# max_list=[]
# min_list=[]
# tmp=[]
# for num in classes:
#     tmp=[]
#     for i in range(10):
#         f=open(data_pth+f"/{num}{i+1}.log","r")
#         text=f.read()
#         text_len=len(re.findall("y",text))
#         tmp.append(text_len)
#     tmp=np.array(tmp)
#     median_list.append(np.median(tmp))
#     mean_list.append(np.mean(tmp))
#     max_list.append(np.max(tmp))
#     min_list.append(np.min(tmp))
# plt_figure(1,median_list,"Median")
# plt_figure(2,mean_list,"Mean")
# plt_figure(3,max_list,"Max")
# plt_figure(4,min_list,"Min")
# plt.show()
