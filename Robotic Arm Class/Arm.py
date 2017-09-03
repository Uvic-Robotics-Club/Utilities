# -*- coding: utf-8 -*-
"""
Created on Tue Aug 01 14:56:44 2017

@author: joell

"""

import numpy as np

class displayArm():
    """
    A class for a 3 link robotic arm with forward and inverse kinematics
    
    
        Parameters
        ----------
            lengths : array_like
                the length of each link in the form [offset, link1, link2, link3]
            units : scalar (optional)
                conversion factor between whatever units the lengths are and meters. For example, if the lengths were given in feet then the conversion factor would be 0.3048
            q1 : scalar (optional)
                inital rotation of the base in radians
            q2 : scalar (optional)
                initial rotation of link 1 from the base in radians
            q3 : scalar (optional)
                intial rotation of link 2 from link 1 in radians
            q4 : scalar (optional)
                initial rotation of link 3 from link 2 in radians
                
        Example
        -------
            >>> arm = displayArm([0.02,3,2,1.5],
                     units=0.3048,
                     q1=45.0/360*2*np.pi,
                     q2=60.0/360*2*np.pi,
                     q3=-30.0/360*2*np.pi,
                     q4=-30.0/360*2*np.pi)
            
        """
    def __init__(self,lengths,*args,**kwargs):
        units = 1.0                     # assume meters
        self.q1= 0.0                    # rotation of the base
        self.q2= 0.0                    # rotation of link 1 from the base
        self.q3= 0.0                    # rotation of link 2 from link 1
        self.q4= 0.0                    # rotation of link 3 from link 2
        
        if(len(args)>= 1):
            units = args[0]
        if(len(args)>= 2):
            self.q1 = args[1]
        if(len(args)>= 3):
            self.q2 = args[2]
        if(len(args)>= 4):
            self.q3 = args[3]
        if(len(args)>= 5):
            self.q4 = args[4]
            
        if(kwargs.has_key("units")):
            self.units = kwargs["units"]
        if(kwargs.has_key("q1")):
            self.q1 = kwargs["q1"]
        if(kwargs.has_key("q2")):
            self.q2 = kwargs["q2"]
        if(kwargs.has_key("q3")):
            self.q3 = kwargs["q3"]
        if(kwargs.has_key("q4")):
            self.q4 = kwargs["q4"]
        
        self.l1=lengths[0]*units        # initial offset from base
        self.l2=lengths[1]*units        # first link
        self.l3=lengths[2]*units        # second link
        self.l4=lengths[3]*units        # third link
        
        self.x_joints = []
        self.y_joints = []
        self.z_joints = []
        
        self.p_joints = []
        # this is to populate x_joints, y_joints, z_joints and p_joints
        self.FK()
    
    def __repr__(self):
        '''
        Returns a string representation of the class. This is what is returned when you type print [displayArm]
        '''
        return '{} (x{:.2f} y{:.2f} z{:.2f})'.format(self.__class__.__name__, self.x_joints[3] ,self.y_joints[3], self.z_joints[3])
        
    def angles(self):
        """
        This function returns the angles from the last time the inverse kimematics function was called.
        If the function has not been called, the function will return zeros.
        
        Parameters
        ----------
            NONE
        Returns
        -------
        q1 : scalar
            angle of the base
        q2 : scalar
            angle that the first joint makes with the base
        q3 : scalar
            angle that the second joint makes with the first
        q4 : scalar
            angle that the third joint makes with the second
                
        >>> return [q1,q2,q3,q4]
        """
        
        return [self.q1,self.q2,self.q3,self.q4]
        
    def angle_from_dot_product(self,a,b):
        '''
        Parameters
        ----------
        a,b : array_like
            1x3 arrays that you want to find the angle betwen
        
        Returns
        -------
        theta : scalar
            the angle from the dot product of a and b
        '''
        a_mag=np.sqrt(np.power(a[0],2)+np.power(a[1],2)+np.power(a[2],2))
        b_mag=np.sqrt(np.power(b[0],2)+np.power(b[1],2)+np.power(b[2],2))
        
        theta=np.arccos(np.power(a_mag*b_mag,-1)*(a[0]*b[0]+a[1]*b[1]+a[2]*b[2]));
        
        return theta
    
    def dh(self,theta,d,a,alpha):
        """
        Computes the denavit-Hartenberg rotation matrix. This essentailly is a rotation around the x and z axis.
        
        
        Perameters
        ----------
        theta : scalar
            rotation around the z axis
        d : scalar
            translation in the z axis
        a : scalar
            translation in the x axis
        alpha : scalar
            rotation around the x axis
        
        
        Returns
        -------
        out : array_like
            Tranformation maxtrix of how to acheive the desired translation and rotation. This array is 4x4.
        
        
        More Information
        ----------------
        More information can be found here https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
        """
        return np.array([[np.cos(theta) , -np.cos(alpha)*np.sin(theta), np.sin(alpha)*np.sin(theta) , a*np.cos(theta)],
                         [np.sin(theta) , np.cos(alpha)*np.cos(theta) , -np.sin(alpha)*np.sin(theta), a*np.sin(theta)],
                         [0             , np.sin(alpha)               , np.cos(alpha)               , d],
                         [0             , 0                           , 0                           , 1.0]])
                         
    def project_along_vector(self,x1,y1,z1,x2,y2,z2,L):
        '''
        Solve for the point px,py,pz that is on a vector with magnitude L away in the direction between point 2 and point 1, starting at point 1
                 
        Parameters
        ----------
            x1, y1, z1 : scalars
                point 1
            x2, y2, z2 : scalars
                point 2
            L : scalar
                Magnitude of vector?
         
        Returns
        -------
            out : array_like
                projected point on the vector
        
        More Information
        ----------------
            Information about what and where this information was pulled from can be found at 
            https://math.oregonstate.edu/home/programs/undergrad/CalculusQuestStudyGuides/vcalc/dotprod/dotprod.html
            and 
            https://en.wikipedia.org/wiki/Vector_projection
        '''
        
        # vector from point 1 to point 2
        vx=x2-x1
        vy=y2-y1
        vz=z2-z1
        v=np.sqrt(np.power(vx,2)+np.power(vy,2)+np.power(vz,2))
        
        ux=vx/v
        uy=vy/v
        uz=vz/v
        
        # Need to always project along radius
        # Project backwards
        px=x1+L*ux
        py=y1+L*uy
        pz=z1+L*uz
        
        return np.array([px,py,pz])
    
    def IK(self,x,y,z,tol_limit=0.05,max_iterations=100):
        '''
        Preforms Inverse Kinematics on the arm using the known lengths and last positions. This uses the FABRIK method. See the more information section for information on the FABRIK method.
        
        
        Parameters
        ----------
        x, y, z : scalars
            desired location
        tol_limit : scalar (optional)
            tolerance between the desired location and actual one after iteration
        max_iterations : scalar (optional)
            maximum itterations that the function is allowed to run
        
        
        Returns
        -------
        q1 : scalar
            rotation of the base about the z axis
        q2 : scalar
            rotation of first link relative to the base
        q3 : scalar
            rotation of the second link relative to the first link
        q4 : scalar
            rotation of the third link relative to the second link
        x_joints : array_like
            x points for all the joint locations
        y_joints : array_like
            y points for all the joint locations
        z_joints : array_like
            z points for all the joint locations
        
        
        More Information
        ----------------
        Overall algorithim: solve for unit vectors going backwards from the desired point to the original point of the joints, then forwards from the origin, see youtube video https://www.youtube.com/watch?v=UNoX65PRehA&t=817s
        
        '''
        
        # Find base rotation    
        
        # returns -pi to pi, different from arctan2
        q1_o=np.arctan2(self.y_joints[-1],self.x_joints[-1])       # Initial angle of where end effector is
        self.q1=np.arctan2(y,x)                               # Desired angle 
        
        base_rotation=self.q1-q1_o                       # Base rotation      

        # Base rotation matrix about z
        R_z=np.array([[np.cos(base_rotation), -np.sin(base_rotation) ,0.0], 
                      [np.sin(base_rotation), np.cos(base_rotation)  ,0.0],
                      [0.0                  , 0.0                    ,1.0]])
     
        # Rotate the location of each joint by the base rotation
        # This will force the FABRIK algorithim to only solve 
        # in two dimensions, else each joint will move as if it has
        # a 3 DOF range of motion
        #print 'inside the fabrik method and x_joints is'
        #print x_joints
        p4=np.dot(R_z,[self.x_joints[3], self.y_joints[3], self.z_joints[3]])
        p3=np.dot(R_z,[self.x_joints[2], self.y_joints[2], self.z_joints[2]])
        p2=np.dot(R_z,[self.x_joints[1], self.y_joints[1], self.z_joints[1]])
        p1=np.dot(R_z,[self.x_joints[0], self.y_joints[0], self.z_joints[0]])
        
        # Store the (x,y,z) position of each joint    
        p4x=p4[0]
        p4y=p4[1]
        p4z=p4[2]
            
        p3x=p3[0]
        p3y=p3[1]
        p3z=p3[2]
                    
        p2x=p2[0]
        p2y=p2[1]
        p2z=p2[2]
            
        p1x=p1[0]
        p1y=p1[1]
        p1z=p1[2]
            
        # store starting point of the first joint
        p1x_o=p1x
        p1y_o=p1y
        p1z_o=p1z
        
        iterations=0
        for q in range(1,max_iterations+1):
            # Make sure the desired x,y,z point is reachable
            if np.sqrt(np.power(x,2)+np.power(y,2)+np.power(z,2))>(self.l2+self.l3+self.l4):
                print ' desired point is likely out of reach'
        
       
            # Overall algorithim: solve for unit vectors going backwards from the
            # desired point to the original point of the joints, then forwards from the
            # origin, see youtube video https://www.youtube.com/watch?v=UNoX65PRehA&t=817s
        
        
            # backwards 
            #project_along_vector(x1,y1,z1,x2,y2,z2,L)
            
            [p3x,p3y,p3z]=self.project_along_vector(x,y,z,p3x,p3y,p3z,self.l4)
            [p2x,p2y,p2z]=self.project_along_vector(p3x,p3y,p3z,p2x,p2y,p2z,self.l3)
            [p1x,p1y,p1z]=self.project_along_vector(p2x,p2y,p2z,p1x,p1y,p1z,self.l2) 
        
            # forwards
        
            [p2x,p2y,p2z]=self.project_along_vector(p1x_o,p1y_o,p1z_o,p2x,p2y,p2z,self.l2)
            [p3x,p3y,p3z]=self.project_along_vector(p2x,p2y,p2z,p3x,p3y,p3z,self.l3)
            [p4x,p4y,p4z]=self.project_along_vector(p3x,p3y,p3z,x,y,z,self.l4)
    
            # Solve for tolerance between iterated point and desired x,y,z,
            tolx=p4x-x
            toly=p4y-y
            tolz=p4z-z
            
            # Make tolerance relative to x,y,z
            tol=np.sqrt(np.power(tolx,2)+np.power(toly,2)+np.power(tolz,2))
    
            iterations=iterations+1
            
            # Check if tolerance is within the specefied limit
            if tol<tol_limit:
                break
    
        # Re-organize points into a big matrix for plotting elsewhere
        self.p_joints= np.array([[p1x, p2x, p3x, p4x],
                                 [p1y, p2y, p3y, p4y],
                                 [p1z, p2z, p3z, p4z]])
                            
        self.x_joints= self.p_joints[0]
        self.y_joints= self.p_joints[1]
        self.z_joints= self.p_joints[2]
        
        
        # Return the joint angles by finding the angles with the dot produvt
        v21=np.array([p2x-p1x, p2y-p1y, p2z-p1z])
        v32=np.array([p3x-p2x, p3y-p2y, p3z-p2z])
        v43=np.array([p4x-p3x, p4y-p3y, p4z-p3z])
        
        # returns -pi to pi
        self.q2=np.arctan2((p2z-p1z), np.sqrt(np.power(p2x-p1x,2)+np.power(p2y-p1y,2)))
        
        # Negative sign because of dh notation, a rotation away from the previous link
        # and towards the x-y plane is a negative moment about the relative z axis.
        # the relative z axis of each link is out of the page if looking at the arm
        # in 2D
        # the x axis in dh convention is typically along the link direction.
        
        self.q3=-1*self.angle_from_dot_product(v21,v32)
        self.q4=-1*self.angle_from_dot_product(v32,v43)
        
        return [self.q1,self.q2,self.q3,self.q4,self.x_joints,self.y_joints,self.z_joints]
        
    def FK(self,*args,**kwargs):
        '''
        Preforms forward kinematics. This uses known angles to compute the resulting end effector location. 
        If this is called with no parameters then the stored values for q1-q4 and l1-l4 are used.
        
        
        Parameters
        ----------
        q1 : scalar (optional)
            angle of the base
        q2 : scalar (optional)
            angle that the first joint makes with the base
        q3 : scalar (optional)
            angle that the second joint makes with the first
        q4 : scalar (optional)
            angle that the third joint makes with the second
        l1 : scalar (optional)
            offset joint of the first link
        l2 : scalar (optional)
            length of the first link
        l3 : scalar (optional)
            length of the second link
        l4 : scalar (optional)
            length of the third link
        
        Returns
        -------
        x_joints : array_like
            x points for all the joint locations
        y_joints : array_like
            y points for all the joint locations
        z_joints : array_like
            z points for all the joint locations
        
        More Information
        ----------------
        More information on forward kinematics can be found here https://en.wikipedia.org/wiki/Forward_kinematics
        '''
        
        # Use forward kinmatics to move robot
        # Initial forward kinematics
        
        q1 = self.q1
        q2 = self.q2
        q3 = self.q3
        q4 = self.q4
        l1 = self.l1
        l2 = self.l2
        l3 = self.l3
        l4 = self.l4
        
        if(len(args)>=1):
            q1 = args[0]
        if(len(args)>=2):
            q2 = args[1]
        if(len(args)>=3):
            q3 = args[2]
        if(len(args)>=4):
            q4 = args[3]
        if(len(args)>=5):
            l1 = args[4]
        if(len(args)>=6):
            l2 = args[5]
        if(len(args)>=7):
            l3 = args[6]
        if(len(args)>=8):
            l4 = args[7]
            
        if(len(kwargs)>0):
            if(kwargs.has_key['q1']):
                q1 = kwargs['q1']
            if(kwargs.has_key['q2']):
                q2 = kwargs['q2']
            if(kwargs.has_key['q3']):
                q3 = kwargs['q3']
            if(kwargs.has_key['q4']):
                q4 = kwargs['q4']
            if(kwargs.has_key['l1']):
                l1 = kwargs['l1']
            if(kwargs.has_key['l2']):
                l2 = kwargs['l2']
            if(kwargs.has_key['l3']):
                l3 = kwargs['l3']
            if(kwargs.has_key['l4']):
                l4 = kwargs['l4']
                  
        T10=self.dh(q1,l1,0,np.pi/2)   # Create transformation matrix from 0 to 1
        T21=self.dh(q2,0,l2,0)      # Create transformation matrix from 2 to 1
        T32=self.dh(q3,0,l3,0)      # Create transformation matrix from 3 to 2
        T43=self.dh(q4,0,l4,0)      # keep q4 constant
        
        T20=np.dot(T10,T21)
        T30=np.dot(T20,T32)
        T40=np.dot(T30,T43)    # Transformation matrix from end effector to the global frame
        
        self.p_joints=[T10[0:3,3],T20[0:3,3],T30[0:3,3],T40[0:3,3]]
        
        self.p_joints=np.transpose(self.p_joints)
           
        self.x_joints= self.p_joints[0]
        self.y_joints= self.p_joints[1]
        self.z_joints= self.p_joints[2]
        
        return [self.x_joints,self.y_joints,self.z_joints]

if(__name__=='__main__'):
    # animated example of how the arm moving around
    arm = displayArm([0.001,3,2,1.5],
                     units=0.3048,
                     q1=45.0/360*2*np.pi,
                     q2=60.0/360*2*np.pi,
                     q3=-30.0/360*2*np.pi,
                     q4=-30.0/360*2*np.pi)
                     
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import axes3d
    import matplotlib.animation as animation
    
    fig = plt.figure("Arm Demo")
    ax = fig.add_subplot(111, projection='3d')
    
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    
    plt.show()
    
    # Time matrix for example helix path
    N_cycles=3
    T=1.0
    
    dt=T/100.0
    Nt=int(round(N_cycles*T/dt))
    t = np.array(range(0,(Nt-1),1))*dt
    
    x=1.7*np.cos(2*np.pi*t+np.pi/8)
    y=1.7*np.sin(2*np.pi*t+np.pi/8)
    z=1.05-.5*t
    
    def animate(n):
        # find angles for x, y, and z
        [q1,q2,q3,q4,x_joints,y_joints,z_joints] = arm.IK(x[n],y[n],z[n])
        
        # clear and plot the data
        ax.clear()
        plt.hold(True)
        ax.plot3D(x_joints,y_joints,z_joints            ,color='b',label='links')
        ax.plot3D(x,y,z                                 ,color='y',label='path')
        ax.scatter3D(x_joints[0],y_joints[0],z_joints[0],color='g',label='p1')
        ax.scatter3D(x_joints[1],y_joints[1],z_joints[1],color='r',label='p2')
        ax.scatter3D(x_joints[2],y_joints[2],z_joints[2],color='m',label='p3')
        ax.scatter3D(x_joints[3],y_joints[3],z_joints[3],color='k',label='p4')
        #ax.scatter3D(0,0,0                              ,color='k')
        ax.legend()
        ax.plot3D([-.5,.5],[ 0,0],[0,0],color='k')
        ax.plot3D([0,0],[ -.5,.5],[0,0],color='k')
        
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')

        ax.grid(True)
        plt.hold(False)
        plt.show()
        
    ani = animation.FuncAnimation(fig,animate, range(1,Nt-1),interval=100)
        
    