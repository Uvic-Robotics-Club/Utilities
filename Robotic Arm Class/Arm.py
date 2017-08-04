# -*- coding: utf-8 -*-
"""
Created on Tue Aug 01 14:56:44 2017

@author: joell

"""
from math import cos,sin,radians,atan2,degrees,pi,sqrt,acos,atan

import numpy as np

class displayArm():
    def __init__(self,lengths,units=1):
        """
        Parameters
            lengths:
                the length of each link in the form [offset, link1, link2, link3]
            units:
                This is the multiplication factor to get your lengths into meters. If not given, defaults to meters.\n
        Example:
            >>> arm = displayArm([0.2, 3, 2, 1.5],0.3048) # 0.3048 is the conversion from feet to meters
            
        """

        self.links = 3
        self.l1=lengths[0]*units        # initial offset from base
        self.l2=lengths[1]*units        # first link
        self.l3=lengths[2]*units        # second link
        self.l4=lengths[3]*units        # third link
        self.q1= 0.0
        self.q2= 0.0
        self.q3= 0.0
        self.q4= 0.0 
        
    def angles(self):
        """
        This function returns the angles from the last time the inverse kimematics function was called.
        If the function has not been called, the function will return zeros.
        
        Parameters
            NONE
        Returns
            array of angles for each link in order.
        Example
            >>> return [q1,q2,q3,q4]
        """
        
        return [self.q1,self.q2,self.q3,self.q4]

    def dh(self,theta,d,a,alpha):
        """
        Perameters
            theta
                rotation around the z axis
            d
                translation in the z axis
            a
                translation in the x axis
            alpha
                rotation around the x axis
                
        Returns
            Tranformation maxtrix of how to acheive the desired translation and rotation 
            
        More Information
            More information can be found here https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
        """
        return np.array([[cos(theta), -cos(alpha)*sin(theta), sin(alpha)*sin(theta) , a*cos(theta)],
                         [sin(theta), cos(alpha)*cos(theta) , -sin(alpha)*sin(theta), a*sin(theta)],
                         [0         , sin(alpha)            ,        cos(alpha)     , d],
                         [0         , 0                     ,          0            , 1]])
                         
            
    def inverseKinematics(self,x,y,z,*args,**kwargs):
        """
        Perameters
            x, y, z
                position of the end effector in cartesian format 
            l1 (optional)
                length of the first joint. If this is not given, will use the lengths given at initialization. This will not override the lengths that were set at initialization.
            l2 (optional)
                length of the second joint. If this is not given, will use the lengths given at initialization. This will not override the lengths that were set at initialization.
        Returns
            q1
                angle that the first joint makes with the base
            q2
                angle that the second joint makes with the first
            q3
                angle that the third joint makes with the second
            >>> return [q1,q2,q3]
            
        More Info
            Talk to max w. for more information about how this works.
        """
        if(len(args)==0):
            if(len(kwargs)==0):
                l1 = self.l1
                l2 = self.l2
            elif(kwargs.has_key('l1') and kwargs.has_key('l2')):
                l1 = kwargs['l1']
                l2 = kwargs['l2']
            else:
                raise ValueError 
        elif(len(args)==2):
            l1 = args[0]
            l2 = args[1]
        else:
            raise ValueError 

        d=sqrt(np.power(x,2)+np.power(y,2))
        rho=sqrt(np.power(d,2) +np.power(z,2))
        
        # cosine law
        gamma = acos(((pow(rho,2)-pow(l1,2)-pow(l2,2))/(-2*l1*l2)))		
        q3=180.0/360*2*pi-gamma
        
        # Rotation of the base
        alpha = atan(z/d)   									
        beta=acos((np.power(l2,2)-np.power(l1,2)-np.power(rho,2))/(-2*l1*rho))
        
        # taken from IK example
        q1=atan(y/x)
        q2=beta+alpha
        q3=-1*q3
        
        self.q2= q1
        self.q3= q2
        self.q4= q3
        
        return [q1,q2,q3]
    
    def forwardKinematics(self,q,lengths=[]):
        """
        Parameters
            q
                array of angles of each link in the format [q1,q2,q3,q4]
                
            lengths (Optional)
                the lengths of each link. If none are given then will use the 
                lengths given at initialization.
                
        Returns
            x_joints
                all the x coordinates for the joints
            y_joints
                all the y coordinates for the joints
            z_joints
                all the z coordinates for the joints
                
            >>> return [x_joints, y_joints, z_joints]
        
        Example
            >>> [x,y,z] = arm.forwardKinematics([pi/4,0,0,pi/4])
            >>> import matplotlib.pyplot as plt
            >>> from mpl_toolkits.mplot3d import Axes3D
            >>> fig = plt.figure(1)
            >>> ax = fig.add_subplot(111, projection='3d')
            >>> ax.plot(x,y,z)
            
        """
        if(len(lengths)==0 or len(lengths)!=4):
            lengths=[self.l1,self.l2,self.l3,self.l4]
            
        #% Initial forward kinematics
        T10=self.dh(q[0],lengths[0],0,pi/2);   #% Create transformation matrix from 0 to 1
        T21=self.dh(q[1],0,lengths[1],0);      #% Create transformation matrix from 2 to 1
        T32=self.dh(q[2],0,lengths[2],0);      #% Create transformation matrix from 3 to 2
        T43=self.dh(q[3],0,lengths[3],0);      #% Create transformation matrix from 4 to 3
        
        #% Transformation matrix from end effector to the global frame
        T20=np.dot(T10,T21);            
        T30=np.dot(T20,T32);
        T40=np.dot(T30,T43);            
        
        #% Initial position of each joint
        p_joints_o=np.array([T10[0:3,3],T20[0:3,3],T30[0:3,3],T40[0:3,3]])
        
        x_joints_o= p_joints_o[:,0];
        y_joints_o= p_joints_o[:,1];
        z_joints_o= p_joints_o[:,2];
    
        return [x_joints_o,y_joints_o,z_joints_o]

    