'''
此文件是加速度计数据校准程序

请将加速度计采集的数据存放在../Data文件夹内，并命名为accel.csv，
其内部需包含静止状态下的加速度计读数，对应表头为：accel_x,accel_y,accel_z，
此外，accel.csv需要有一列pose，用A1～A6标记六个静止姿势。
'''
import csv
import numpy as np
from pathlib import Path
from collections import defaultdict

calibration_dir=Path(__file__).resolve().parent.parent
accelcsv_path=calibration_dir/"Data"/"accel.csv"

class Obj:
    '''用于目标函数的迭代计算'''
    def __init__(self,data):
        '''
        参数说明：
        data：采集的数据，形状为（6，3），注意其内部的数据需要已经归一化，即以g为单位
        '''
        self.data=data
        self.params=np.array([0,0,0,1,1,1])     #分别代表[bx,by,bz,sx,sy,sz]，其中b为零偏误差，s为刻度因数误差
        self.r=np.zeros((6,))
        self.cal_r()
        self.jacobian=np.zeros((6,6))
        self.cal_jacobian()
        self.ok=False
    def cal_r(self):
        tmp=self.data-self.params[:3]
        tmp=tmp*tmp
        tmp*=self.params[3:]**2
        self.r=np.sum(tmp,axis=1)-1
    def cal_jacobian(self):
        tmp1=self.data-self.params[:3]
        tmp2=tmp1.copy()
        tmp2=tmp2**2
        tmp1=(-2)*(self.params[3:]**2)*tmp1
        tmp2=2*self.params[3:]*tmp2
        self.jacobian=np.concatenate([tmp1,tmp2],axis=1)
    def update(self):
        delta,_,rank,_=np.linalg.lstsq(self.jacobian,-self.r,rcond=None)
        if rank<6:
            raise ValueError("rank<6")
        self.params=self.params+delta
        self.cal_r()
        self.cal_jacobian()
        bias_step=np.max(np.abs(delta[:3]))
        scale_step=np.max(np.abs(delta[3:]))
        if bias_step<0.1/16384.0 and scale_step<1e-5:
            self.ok=True
        
raw_data=defaultdict(list)

with accelcsv_path.open(encoding="utf-8-sig",newline="") as f:
    reader=csv.DictReader(f)
    for row in reader:
        raw_data[row['pose']].append([int(row['accel_x']),int(row['accel_y']),int(row['accel_z'])])
data=np.zeros((6,3))
for idx in range(6):
    key=f"A{idx+1}"
    if not raw_data[key]:
        raise ValueError(f"姿势{key}缺少有效数据用于计算")
    tmp=np.column_stack(raw_data[key])
    data[idx,:]=np.sum(tmp,axis=1)/(tmp.shape[1]*16384.0)
obj=Obj(data)
for i in range(1000):
    obj.update()
    if obj.ok:
        break
if not obj.ok:
    print("最大迭代次数内未满足停止迭代条件，校准失败")
else:
    print("加速度计零偏结果：")
    print(f"{{{obj.params[0]:.6f}f,{obj.params[1]:.6f}f,{obj.params[2]:.6f}f}}")
    print("加速度计刻度因数结果：")
    print(f"{{{obj.params[3]:.6f}f,{obj.params[4]:.6f}f,{obj.params[5]:.6f}f}}")
