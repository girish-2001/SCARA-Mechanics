from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import math

class scara:
    def __init__(self,l1,l2,v):
        #initializing some variables
        self.v = v #desired end-effector velocity
        self.l1 = l1
        self.l2 = l2
        self.beta = 0.05 #beta value for reducing the error reduce the step size
        self.damping = 0.0 #damping factor for DLSIK
        self.points_x = []
        self.points_y = []
        self.error = []
        self.iteration = []
        self.actual_vel = []
        self.theta_1_i = 0 #initial shoulder angle
        self.theta_2_i = 0.001 #initial elbow angle
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
        plt.figure(1)
        plt.title("Tracking error")
        plt.plot(x_points,y_points)
    
    def main_func(self):
        #self.theta_1_i = float(input("Enter initial shoulder angle = "))*math.pi/180
        #self.theta_2_i = float(input("Enter initial elbow angle = "))*math.pi/180
        #self.plot(self.theta_1_i,self.theta_2_i)
        print("Current point = ",self.FK(self.theta_1_i,self.theta_2_i))
        #self.x_g = float(input("Enter x-coordinate of goal = "))
        #self.y_g = float(input("Enter y-coordinate of goal = "))
        g = np.array([[self.x_g,self.y_g]]) #goal vector
        self.theta_1 = self.theta_1_i
        self.theta_2 = self.theta_2_i

        for self.damping in np.arange(0.1,2.6,0.1):
            print("damping coefficient=",self.damping)
            self.theta_1 = self.theta_1_i
            self.theta_2 = self.theta_2_i
            e = self.FK(self.theta_1,self.theta_2)
            self.points_x.append(e[0][0])
            self.points_y.append(e[0][1])
            self.T = np.linalg.norm(g-e)/self.v
            area = 0
            #del_t = self.beta*self.T
            #print("Time required = ",self.T)
            #vel = (g-e)/self.T
            #print ("velocity vector = ",vel)
            #print("initial error is:",np.linalg.norm(g-e))
            omega_list= []
            while (np.linalg.norm(g-e)>0.01):
                #print('n=',self.n)
                J,J_inv,J_t = self.compute_jacobian(self.theta_1,self.theta_2)
                self.iteration.append(self.n)
                self.error.append(np.linalg.norm(g-e))
                
                self.n+=1
                self.T = np.linalg.norm(g-e)/self.v
                vel = (g-e)/self.T
                del_t = self.beta*self.T
                self.t+=del_t
                matrix = np.matmul(J_t,np.linalg.inv(np.matmul(J,J_t)+self.damping*self.damping*np.identity(2)))
                omega = np.matmul(matrix,np.transpose(vel))
                actual_vel = np.matmul(J,omega)
                self.actual_vel.append(np.linalg.norm(np.transpose(actual_vel)))
                self.omega_1.append(omega[0][0])
                self.omega_2.append(omega[1][0])
                self.theta_1 += omega[0][0]*del_t
                self.theta_2 += omega[1][0]*del_t
                self.time.append(self.t)
                e = self.FK(self.theta_1,self.theta_2)
                #if np.linalg.norm(e)<1:

                omega_list.append(np.linalg.norm(np.transpose(omega)))


                self.points_x.append(e[0][0])
                self.points_y.append(e[0][1])
                area += abs(self.points_x[self.n]-self.points_x[self.n-1])*abs(self.points_y[self.n]/2+self.points_y[self.n-1]/2)


                #plot(theta_1,theta_2)
                #print("error is:",np.linalg.norm(g-e))
            #self.damping+=1
            self.omega_max.append(max(omega_list))
            self.tracking_error.append(area)
            self.lambda_1.append(self.damping)
            #print("Final angles are:",self.theta_1*180/math.pi,self.theta_2*180/math.pi)
            #print("final coordinates:",e)
            #self.plot(self.theta_1,self.theta_2)
            #plt.figure(2)
            #plt.axis("scaled")
            #plt.title("velocity")
            #plt.plot(self.time,self.actual_vel)
            #plt.figure(1)
            #plt.scatter(self.points_x,self.points_y)
            #plt.figure(3)
            #plt.title("omega_1")
            #plt.axis("scaled")
            #plt.plot(self.time,self.omega_1)
            #plt.figure(4)
            #plt.title("omega_2")
            #plt.axis("scaled")
            #plt.plot(self.time,self.omega_2)
            #plt.legend(["omega-1","omega-2"])

            #plt.show()
        plt.figure(2)
        #plt.title("Tracking error")
        plt.plot(self.lambda_1,self.tracking_error)
        plt.plot(self.lambda_1,self.omega_max)
        plt.legend(["tracking error","peak velocity"])
        plt.show()
            
        


if __name__ == "__main__":
    robo = scara(1,1,0.5)
    robo.main_func()







    