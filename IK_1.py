from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import math

from numpy.core.defchararray import count

#Code for IK for a 2-axis planar robotic arm (SCARA)
#Link lengths: l1 = l2 = 210 mm
#Inverse Kinematics is computed using the Jacobian matrix

#Function for forward kinematics, input the two joint angles
#plt.axis("scaled")
l1 = 1
l2 = 1
beta = 0.05
error = []
iteration = []
def FK(theta_1,theta_2):
    x = l1*math.cos(theta_1) + l2*math.cos(theta_1+theta_2)
    y = l1*math.sin(theta_1) + l2*math.sin(theta_1+theta_2)
    e = np.array([[x,y]])
    return e

#Function for computing jacobian and its inverse
def compute_jacobian(theta_1,theta_2):
    j11 = -l1*math.sin(theta_1)-l2*math.sin(theta_1+theta_2)
    j12 = -l2*math.sin(theta_1+theta_2)
    j21 = l1*math.cos(theta_1)+l2*math.cos(theta_1+theta_2)
    j22 = l2*math.cos(theta_1+theta_2)
    j = np.array([[j11,j12],[j21,j22]])
    j_inv = np.linalg.pinv(j)
    return j,j_inv

def plot(theta_1,theta_2):
    end = FK(theta_1,theta_2)
    intermediate_x = l1*math.cos(theta_1)
    intermediate_y = l1*math.sin(theta_1)
    x_points = np.array([0,intermediate_x,end[0][0]])
    y_points = np.array([0,intermediate_y,end[0][1]])
    plt.figure(1)
    plt.title("SCARA")
    plt.plot(x_points,y_points)
    

if __name__ == "__main__":
    #initial angles are 10 degrees each
    theta_1 = 10*math.pi/180
    theta_2 = -10*math.pi/180
    print("current point:",FK(theta_1,theta_2))
    plot(theta_1,theta_2)
    n=1
    x_g = float(input("Enter x-coordinate of goal = "))
    y_g = float(input("Enter y-coordinate of goal = "))
    g = np.array([[x_g,y_g]]) #goal vector
    if l1-l2 <= (math.sqrt(math.pow(x_g,2)+math.pow(y_g,2))) <= l1+l2:
        e = FK(theta_1,theta_2)
        print("initial error is:",np.linalg.norm(g-e))
        while (np.linalg.norm(g-e)>0.1):
            J,J_inv = compute_jacobian(theta_1,theta_2)
            iteration.append(n)
            error.append(np.linalg.norm(g-e))
            n+=1
            del_e = beta*(g-e)
            del_theta = np.matmul(J_inv,np.transpose(del_e))
            theta_1 += del_theta[0][0]
            theta_2 += del_theta[1][0]
            e = FK(theta_1,theta_2)
            #print("error is:",np.linalg.norm(g-e))
        print("Final angles are:",theta_1*180/math.pi,theta_2*180/math.pi)
        print("final coordinates:",e)
        plot(theta_1,theta_2)
        plt.figure(2)
        #plt.axis("scaled")
        plt.title("Error")
        plt.plot(iteration,error)
        plt.show()
        
    else:
        print("configuration not possible")



    
    
