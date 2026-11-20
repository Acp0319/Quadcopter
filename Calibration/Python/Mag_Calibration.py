'''
此文件是磁力计数据校准程序

请将磁力计采集的数据存放在../Data文件夹内，并命名为mag.csv，
其内部需包含多方向转动时的磁力计读数，对应表头为：mag_x,mag_y,mag_z，
此外，mag.csv需要有一列mag_timestamp，标识每一组磁力计数据的时间戳。
'''
import csv
import numpy as np
from pathlib import Path
from collections import defaultdict

calibration_dir=Path(__file__).resolve().parent.parent
magcsv_path=calibration_dir/"Data"/"mag.csv"

seen=set()
raw_data=[]

with magcsv_path.open(encoding="utf-8-sig",newline="") as f:
    reader=csv.DictReader(f)
    for row in reader:
        key=row['mag_timestamp'].strip()
        if key in seen:
            continue
        seen.add(key)
        raw_data.append([float(row['mag_x']),float(row['mag_y']),float(row['mag_z'])])
if not raw_data:
    raise ValueError("没有有效的样本用于计算")
data=np.column_stack(raw_data).T    #(N,3)
N=data.shape[0]
c=np.sum(data,axis=0)/N
s=np.sqrt((np.sum(np.square(data-c)))/N)
if s==0:
    raise ValueError("所有样本完全相同，无法拟合")
processed_data=(data-c)/s

pro_x=processed_data[:,0:1]
pro_y=processed_data[:,1:2]
pro_z=processed_data[:,2:3]
D=(np.concatenate([processed_data**2,2*pro_x*pro_y,2*pro_x*pro_z,2*pro_y*pro_z,
                   processed_data],axis=1))         #D[i]=[xi^2,yi^2,zi^2,2*xi*yi,2*xi*zi,2*yi*zi,xi,yi,zi]
p,_,rank,_=np.linalg.lstsq(D,np.ones(N),rcond=None) #p=[c11,c22,c33,c12,c13,c23,d1,d2,d3]
if rank<9:
    raise ValueError("rank<9")
C=np.zeros((3,3))
d=p[6:].copy()
C[0][0],C[1][1],C[2][2]=p[0],p[1],p[2]
C[0][1],C[0][2],C[1][2],C[1][0],C[2][0],C[2][1]=p[3],p[4],p[5],p[3],p[4],p[5]
b_u=-0.5*np.linalg.solve(C,d)   #b_u=-0.5*C^(-1)*d
k=1+b_u.T@C@b_u
Q_u=C/k
eigenvalues,eigenvectors=np.linalg.eigh(Q_u)
if np.any(eigenvalues<=0):
    raise ValueError("Q_u不是正定矩阵，请检查数据和拟合结果")
sqrt_lambda=np.diag(np.sqrt(eigenvalues))
A_u=eigenvectors@sqrt_lambda@eigenvectors.T

b=c+s*b_u
A=A_u/s

print("磁力计数据矫正偏置b:")
print(f"{{{b[0]:.9e}f,{b[1]:.9e}f,{b[2]:.9e}f}}")
print("磁力计数据矫正矩阵A:")
print(f"{{{{{A[0][0]:.9e}f,{A[0][1]:.9e}f,{A[0][2]:.9e}f}},{{{A[1][0]:.9e}f,{A[1][1]:.9e}f,{A[1][2]:.9e}f}},{{{A[2][0]:.9e}f,{A[2][1]:.9e}f,{A[2][2]:.9e}f}}}}")

test_data=data.T    #(3,N)
test_result=A@(test_data-b[:,None])
test_norms=np.linalg.norm(test_result,axis=0)
test_mean=np.mean(test_norms)
test_std=np.std(test_norms)
test_max_dev=np.max(np.abs(test_norms-1))
print("\n\n\n")
print("使用这组偏置b和矩阵A的测试结果：")
print(f"校正后模长均值：{test_mean:.6f}")
print(f"校正后模长标准差：{test_std:.6f}")
print(f"校正后模长偏离1的最大偏移量：{test_max_dev:.6f}")