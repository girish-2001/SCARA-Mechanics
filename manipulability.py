from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import math

from numpy.matrixlib.defmatrix import matrix

class scara:
    def __init__(self,l1,l2):
        #initializing some variables
        #self.v = v #desired end-effector velocity
        self.l1 = l1
        self.l2 = l2
        self.beta = 0.05 #beta value for reducing the error reduce the step size
        self.damping = 0.0 #damping factor for DLSIK
        self.points_x = []
        self.points_y = []
        self.error = []
        self.iteration = []
        self.actual_vel = []
        #self.theta_1_i = 0 #initial shoulder angle
        #self.theta_2_i = 0.001 #initial elbow angle
        self.theta_1 = 0 #shoulder angle
        self.theta_2 = 0 #elbow angle
        self.x_g = -2 #x-coordinate of goal
        self.y_g = 0 #y-coordinate of goal
        self.n = 0
        self.T = 0 #Total time taken
        self.t = 0
        self.omega_1 = []
        self.omega_2 = []
        self.time = []
        self.n = 0
        self.tracking_error = []
        self.lambda_1 = []
        self.omega_max = []

    #Function for forward kinematics - Computes end effector position with theta's as input parameters
    def FK(self,theta_1,theta_2):
        x = self.l1*math.cos(theta_1) + self.l2*math.cos(theta_1+theta_2)
        y = self.l1*math.sin(theta_1) + self.l2*math.sin(theta_1+theta_2)
        e = np.array([[x,y]])
        return e
    #Function for computing the Jacobian, its transpose and its pseudo-inverse
    def compute_jacobian(self,theta_1,theta_2):

        j11 = -self.l1*math.sin(theta_1)-self.l2*math.sin(theta_1+theta_2)
        j12 = -self.l2*math.sin(theta_1+theta_2)
        j21 = self.l1*math.cos(theta_1)+self.l2*math.cos(theta_1+theta_2)
        j22 = self.l2*math.cos(theta_1+theta_2)
        j = np.array([[j11,j12],[j21,j22]])
        j_inv = np.linalg.pinv(j)
        jt = np.transpose(j)
        return j,j_inv,jt

    #Plot the SCARA links given the joint angles as input parameters
    def plot(self,theta_1,theta_2):
        end = self.FK(theta_1,theta_2)
        intermediate_x = self.l1*math.cos(theta_1)
        intermediate_y = self.l1*math.sin(theta_1)
        x_points = np.array([0,intermediate_x,end[0][0]])
        y_points = np.array([0,intermediate_y,end[0][1]])
        plt.figure(2)
        #plt.title("Tracking error")
        plt.plot(x_points,y_points,color="orange")
    
    def main_func(self):
        self.theta_1= float(input("Enter shoulder angle = "))*math.pi/180
        self.theta_2 = float(input("Enter elbow angle = "))*math.pi/180
        #self.plot(self.theta_1_i,self.theta_2_i)
        print("Current point = ",self.FK(self.theta_1,self.theta_2))
        #self.x_g = float(input("Enter x-coordinate of goal = "))
        #self.y_g = float(input("Enter y-coordinate of goal = "))
        #g = np.array([[self.x_g,self.y_g]]) #goal vector
        #self.theta_1 = self.theta_1_i
        #self.theta_2 = self.theta_2_i
        i_list = []
        j_list = []
        vx = []
        vy = []
        for i in np.arange(-1,1,0.01):
            j = math.sqrt(1-i*i)
            J,J_inv,Jt = self.compute_jacobian(self.theta_1,self.theta_2)
            omega_1 = np.array([[i],[j]])
            
            v1 = np.matmul(J,omega_1)
            
            vx.append(v1[0][0])
            vy.append(v1[1][0])
            
            i_list.append(i)
            
            j_list.append(j)
        
        for i in np.arange(-1,1,0.01):
            j = -math.sqrt(1-i*i)
            J,J_inv,Jt = self.compute_jacobian(self.theta_1,self.theta_2)
            omega_2 = np.array([[i],[j]])
            v2 = np.matmul(J,omega_2)
            vx.append(v2[0][0])
            vy.append(v2[1][0])
            i_list.append(i)
            j_list.append(j)



        matrix = np.matmul(J,Jt)
        value, v = np.linalg.eig(matrix)
        print("eigenvalues are = ",value)
        print("eigen vectors are = ",v)
        major_axis = math.sqrt(max(value))
        minor_axis = math.sqrt(min(value))
        print("major axis length = ", major_axis)
        print("minor axis length = ", minor_axis)
        print("condition number k = ", minor_axis/major_axis)
        print("Manipulability index = ",math.sqrt(np.linalg.det(matrix)))
        plt.figure(1)
        plt.title("Angular velocity circle")
        plt.plot(i_list,j_list)
        plt.figure(2)
        plt.title("Manipulability ellipse")
        plt.plot(vx,vy,color="red")
        self.plot(self.theta_1,self.theta_2)
        #plt.legend(["tracking error","peak velocity"])
        plt.axis("scaled")
        plt.show()
            
        


if __name__ == "__main__":
    robo = scara(1,1)
    robo.main_func()







    