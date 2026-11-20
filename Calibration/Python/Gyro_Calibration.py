'''
此文件是陀螺仪数据校准程序

请将陀螺仪采集的数据存放在../Data文件夹内，并命名为gyro.csv，
其内部需包含静止状态下的陀螺仪读数，对应表头为：gyro_x,gyro_y,gyro_z。
'''
import csv
import numpy as np
from pathlib import Path

calibration_dir=Path(__file__).resolve().parent.parent
gyrocsv_path=calibration_dir/"Data"/"gyro.csv"

data=[]

with gyrocsv_path.open(encoding="utf-8-sig",newline="") as f:
    reader=csv.DictReader(f)
    for row in reader:
        data.append([int(row['gyro_x']),int(row['gyro_y']),int(row['gyro_z'])])
if not data:
    raise ValueError("没有有效的样本用于计算")
arr=np.column_stack(data)
result=np.sum(arr,axis=1)/arr.shape[1]
print("陀螺仪零偏结果：")
print(f"{{{result[0]:.2f}f,{result[1]:.2f}f,{result[2]:.2f}f}}")