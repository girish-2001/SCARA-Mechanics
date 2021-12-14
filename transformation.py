from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import math

fig = plt.figure()
ax = plt.axes(projection ='3d')
v_in= [np.array([[1],[1],[0]]),np.array([[-1],[1],[0]]),np.array([[-1],[-1],[0]]),np.array([[1],[-1],[0]])
,np.array([[1],[1],[1]]),np.array([[-1],[1],[1]]),np.array([[-1],[-1],[1]]),np.array([[1],[-1],[1]])]

#Function for plotting the cube
def plot(v,colour):
    x = np.zeros(8)
    y = np.zeros(8)
    z = np.zeros(8)
    for i in range(len(v)):
        x[i] = v[i][0][0]
        y[i] = v[i][1][0]
        z[i] = v[i][2][0]
    if colour==1:
        kwargs = {'alpha': 1, 'color': 'red'}
    else:
        kwargs = {'alpha': 1, 'color': 'green'}
    ax.plot3D([x[0],x[1],x[2],x[3],x[0]],[y[0],y[1],y[2],y[3],y[0]],[z[0],z[1],z[2],z[3],z[0]],**kwargs)
    ax.plot3D([x[4],x[5],x[6],x[7],x[4]],[y[4],y[5],y[6],y[7],y[4]],[z[4],z[5],z[6],z[7],z[4]],**kwargs)
    ax.plot3D([x[0],x[4]],[y[0],y[4]],[z[0],z[4]],**kwargs)
    ax.plot3D([x[1],x[5]],[y[1],y[5]],[z[1],z[5]],**kwargs)
    ax.plot3D([x[2],x[6]],[y[2],y[6]],[z[2],z[6]],**kwargs)
    ax.plot3D([x[3],x[7]],[y[3],y[7]],[z[3],z[7]],**kwargs)

def transform(v):

    roll = float(input("Enter roll = "))
    pitch = float(input("Enter pitch = "))
    yaw = float(input("Enter yaw = "))
    x_d = float(input("Enter x displacement = "))
    y_d = float(input("Enter y displacement ="))
    z_d = float(input("Enter z-displacement = "))

    roll*= math.pi/180
    pitch*= math.pi/180
    yaw*= math.pi/180
    v_o = []
    v_r = []
    v_r_trial = []
    c2 = math.cos(pitch)
    s2 = math.sin(pitch)
    c1 = math.cos(roll)
    s1 = math.sin(roll)
    c3 = math.cos(yaw)
    s3 = math.sin(yaw)

    Rx = np.array([[1,0,0],
                 [0,c1,-s1],
                 [0,s1,c1]])
    Ry = np.array([[c2,0,s2],
                  [0,1,0],
                  [-s2,0,c2]])
    Rz = np.array([[c3,-s3,0],
                  [s3,c3,-0],
                   [0,0,1]])
    
    

    print("Rx = ",Rx, "Ry= ",Ry,"Rz= ", Rz)

    R1 = np.dot(Rz,Ry)
    R1 = np.dot(R1,Rx)
    
    Tr = np.array([[1,0,0,x_d],
                    [0,1,0,y_d],
                    [0,0,1,z_d],
                    [0,0,0,1]])
    print("Tr= ",Tr)
    
    
    for i in range(len(v)):
        v_r.append(np.matmul(R1,v[i]))
        v_r[i] = np.append(v_r[i],np.array([[1]]),axis=0)
        #v_o.append(np.dot(Tr,v_r[i]))

        #v1 = np.dot(R1,v[i])
        #v_r.append(v1)
        #v1 = np.append(v1,np.array([[1]]),axis=0)
        
        

        #print(v1)
        #v_o.append(np.dot(Tr,v1))
        
        
    print("vr= ", v_r)
    for j in range(len(v)):
        v_o.append(np.matmul(Tr,v_r[j]))
        #v_o[j] = np.delete(v_o[j],3,axis=0)
        #v_r[j] = np.delete(v_r[j],3,axis=0)
    #print("updates vr = ",v_r)
    print("vo= ",v_o)
    #plot(v_r,2)
    #plot(v_o,1)

    return v_o




if __name__ == "__main__":
    #v_final,v_rot,
    v_final= transform(v_in)
    #print("v_rot = ",v_rot)
    #print("vo= ", v_final)
    
    plot(v_in,1)
    #plot(v_rot,2)
    #plot(trial,2)
    plot(v_final,2)
    plt.show()
        
    
    