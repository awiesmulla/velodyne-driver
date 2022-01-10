import numpy as np
import matplotlib.pyplot as plt

for j in range(100):
    if j < 99:
        continue
    data = np.load("/media/amm/Backup/acads/ARTPARK/test/transvahan/Data/2021-12-09/run1_lidar/%05d.npy"%(j))
    #data = data.astype('int16')
    lidar=[]
    alpha=(0)
    rot = np.array([[np.cos(alpha),np.sin(alpha)],
                    [-np.sin(alpha),np.cos(alpha)]])
    for i in range(data.shape[0]):
        temp=np.array([data[i,0],data[i,1]])
        dist=np.linalg.norm(temp)
        if dist<=1750:
            lidar.append(np.dot(temp,rot))
    
    #data = data/10
    lidar=np.array(lidar)
    plt.scatter(lidar[:,0],lidar[:,1],s=0.1)
    #plt.savefig("test/%05d.png"%(j))
    #plt.clf()
    plt.show()
    print(j)

img = plt.imread("maps/farm.png")
#lidar=np.array(lidar)
#plt.imshow(img)
#plt.scatter(data[:,1]+4800,data[:,0]+1000,s=0.1)
#plt.scatter(lidar[:,0],lidar[:,1],s=0.1)
#plt.show()
