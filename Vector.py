# -*- coding: utf-8 -*-
"""
Created on Fri Aug 11 16:29:18 2017

@author: joell
"""
import math

class Vector():
    """
        Vector class: 
            Representing a vector in 3D space.
        
        Can accept formats of:
            Cartesian coordinates in the x, y, z space.
            Spherical coordinates in the r, theta, phi space.
            Cylindrical coordinates in the r, theta, z space.
    
    
        Parameters:
            args:
                Can accept 0,1,2,3, or 6 arguments defining the vector.
                The first 0,1,2,3 arguments must define the ending point of the vector.
                
                If 6 arguments are used then the first 3 are used to define 
                the end of the vector and the last 3 are used to define the start of the vector in (x,y,z).
                
                For the args length of 3 the space must be defined as spherical,
                cylindrical, or if none are specified its assumed to be cartesian.
            
            kwargs:
                Spherical
                    r, theta, and phi in that order
                Cylindrical
                    r, theta, z in that order
                Cartesian
                    x, y, z in that order
         
        Returns:
            The A vector object going from the origin to the point specied
            
        Examples:
        
        >>> Vector.Vector()      #creates a vector going from (0,0,0) -> (0,0,0)
        >>> Vector.Vector(1)     #creates a vector going from (0,0,0) -> (1,0,0)
        >>> Vector.Vector(1,2)   #creates a vector going from (0,0,0) -> (1,2,0)
        >>> Vector.Vector(1,2,3) #creates a vector going from (0,0,0) -> (1,2,3)
        >>> Vector.Vector(1,2,3,5,4,3) #creates a vector going from (5,4,3) -> (1,2,3)
        >>> Vector.Vector(1,1,1,cylindrical=True) #creates a vector going from (0,0,0) -> (0.540302305868, 0.841470984808, 1)
    """
    
    def __init__(self,*args, **kwargs):
        
        self.x_origin = 0.0
        self.y_origin = 0.0
        self.z_origin = 0.0
        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        
        
        if len(args)==1:
            self.x = float(args[0])
            self.y = 0.0
            self.z = 0.0
        elif len(args)==2:
            self.x = float(args[0])
            self.y = float(args[1])
            self.z = 0.0
        elif len(args)==3 or len(args)==6:
            if(kwargs.has_key('spherical')):
                if(kwargs['spherical']!=False):
                    self.x = args[0]*math.sin(args[1])*math.cos(args[2])
                    self.y = args[0]*math.sin(args[1])*math.sin(args[2])
                    self.z = args[0]*math.cos(args[1])
                    
            elif(kwargs.has_key('cylindrical')):
                if(kwargs['cylindrical']!=False):
                    self.x = args[0]*math.cos(args[1])
                    self.y = args[0]*math.sin(args[1])
                    self.z = float(args[2])
            else:
                self.x = float(args[0])
                self.y = float(args[1])
                self.z = float(args[2])
                
            if(len(args)==6):
                self.x_origin = float(args[3])
                self.y_origin = float(args[4])
                self.z_origin = float(args[5])
        
        
        #super(Vector, self).__init__(x, y, z)
        
    def __add__(self,anotherVector):
        """
        Adding two vectors together.
        
        This is done in tip to tail method. If either (or both) vectors are not 
        from the origin then it moves them to the orign. If the vector being based
        in is scalar (ie. type of float, int, or long) then the value will be added
        to x, y, and z onto the end; maintaining the origin if it is non-zero.
        
                
        """
        if(type(anotherVector)==type(self)):
            return Vector(
                (self.x-self.x_origin) + (anotherVector.x-anotherVector.x_origin),
                (self.y-self.y_origin) + (anotherVector.y-anotherVector.y_origin),
                (self.z-self.z_origin) + (anotherVector.z-anotherVector.z_origin))
            
        elif(type(anotherVector)in [float,int,long]):
            return Vector(self.x+anotherVector,
                          self.y+anotherVector,
                          self.z+anotherVector,
                          self.x_origin,
                          self.y_origin,
                          self.z_origin)
        else:
            raise TypeError, "Tried to add a %s type to a vector. Requres a vector object, int, float, or long"%(type(anotherVector))
        
    def __sub__(self,anotherVector):
        """
        subtracting two vectors .
        
        This is done in tip to tail method. If either (or both) vectors are not 
        from the origin then it moves them to the orign. If the vector being based
        in is scalar (ie. type of float, int, or long) then the value will be subtracted
        to x, y, and z onto the end; maintaining the origin if it is non-zero.
        
                
        """
        if(type(anotherVector)==type(self)):
            return Vector(
                (self.x-self.x_origin) - (anotherVector.x-anotherVector.x_origin),
                (self.y-self.y_origin) - (anotherVector.y-anotherVector.y_origin),
                (self.z-self.z_origin) - (anotherVector.z-anotherVector.z_origin))
            
        elif(type(anotherVector)in [float,int,long]):
            return Vector(self.x-anotherVector,
                          self.y-anotherVector,
                          self.z-anotherVector,
                          self.x_origin,
                          self.y_origin,
                          self.z_origin)
        else:
            raise TypeError, "Tried to subtract a %s type to a vector. Requres a vector object, int, float, or long"%(str(type(anotherVector)))
        
    def __mul__(self,anotherVector):
        """ Return a Vector instance as the cross product of two vectors """
        return self.cross(anotherVector)
    
    def __str__(self):
        """
        Returns the string representation of the vector.
        
        This is the one that is used by print
        
        The format that is set is (x_origin,y_origin,z_origin) -> (x,y,z)
        
        >>> v1 = Vector(1,2,3)
        >>> print v1
        >>> '(0,0,0) -> (1,2,3)'
        """
        return "({1}{0}{2}{0}{3}) -> ({4}{0}{5}{0}{6})".format(",",self.x_origin,self.y_origin,self.z_origin,self.x,self.y,self.z)
    
    def __repr__(self):
        """
        Returns the string representation of the vector with the class name.
        
        
        The format that is set is 'Vector object: (x_origin,y_origin,z_origin) -> (x,y,z)'
        
        >>> v1 = Vector(1,2,3)
        >>> print repr(v1)
        >>> 'Vector Object: (0,0,0) -> (1,2,3)'
        """
        return "Vector object: ({1}{0}{2}{0}{3}) -> ({4}{0}{5}{0}{6})".format(",",self.x_origin,self.y_origin,self.z_origin,self.x,self.y,self.z)
        
    def __getitem__(self,index):
        """
        Allows for index notation in the vector.
        
        Returns:
            the x,y, or z component of the vector. This IS from the modified origin
        
        >>> v1 = Vector(1,2,3)
        >>> print v1[0]
        >>> '2'
            
        """
        assert type(index) is int, "index is not an integer"
        assert index in [0,1,2], "invalid index values are 0,1,2"
        
        if(index==0):
            return self.x - self.x_origin
        elif(index==1):
            return self.y - self.y_origin
        else:
            return self.z - self.z_origin
   

    def magnitude(self):
        """
        Return magnitude of the vector.
        
        calculated as the square root of the sum of each element squared.
        
        sqrt(x^2+y^2+z^2)
        
        """

        return math.sqrt(pow(self.x-self.x_origin,2)+pow(self.y-self.y_origin,2)+pow(self.z-self.z_origin,2))

    def add(self, vector):
        """
        Adding two vectors together.
        
        This is done in tip to tail method. If either (or both) vectors are not 
        from the origin then it moves them to the orign. If the vector being based
        in is scalar (ie. type of float, int, or long) then the value will be added
        to x, y, and z onto the end; maintaining the origin if it is non-zero.
        
                
        """

        return self.__add__(vector)


    def dot(self, vector):
        """ 
        Return the dot product of two vectors.
        
        a.dot(b) = a1*b1+a2*b2+a3*b3
        
        Parameters:        
            vector:
                a vector object to preform the dot product with
        
        Returns:
            a number representing the dot product of vector passed in and the 
            current vector.
            
        
        """
        assert type(vector) is type(self), "tried to take the dot product with a %s type"%(str(type(vector)))
        
        return (self.x*vector.x+self.y*vector.y+self.z*vector.z)

    def cross(self, vector):
        """
        Return a Vector instance as the cross product of two vectors
        
        """
        
        assert type(vector) is type(self), "Tried to take the cross product of %s type object, only vectors"%(str(type(vector)))
        
        return Vector((self.y * vector.z - self.z * vector.y),
                      (self.z * vector.x - self.x * vector.z),
                      (self.x * vector.y - self.y * vector.x))

    def angle(self, vector):
        """
        Return the angle between two vectors in radians.

        theta = acos((u(dot)v)/(||u||*||v||)) 
        
        u (dot) v is the dot product between u and v
        
        ||u|| is the length (magnitude) of the vector
        """

        return math.acos(self.dot(vector)/(self.magnitude()*vector.magnitude()))

    def parallel(self, vector):
        """ Return True if vectors are parallel to each other. """

        return True if self.cross(vector).magnitude() == 0 else False

    def perpendicular(self, vector):
        """ Return True if vectors are perpendicular to each other. """

        return True if self.dot(vector) == 0 else False

    
    
    def rotate(self,angle,axis=(0,0,1)):
        """
        Angle is the amount of rotation in radians.
        
        Axis is a tuple or array indicating which axis to rotate around. (x,y,z).
        If more than 1 axis is specified then it will rotate around in the order z,y,x.
        
        Returns a rotated vector with rotated origin.
        
        """
        
        if(type(axis[0])!=int or type(axis[1])!=int or type(axis[2])!=int):
            raise ValueError, "cannot rotate on a non integer axis, must be in the format of (int,int,int)"
            
        x = self.x
        y = self.y
        z = self.z
        
        x_o = self.x_origin
        y_o = self.y_origin
        z_o = self.z_origin
        
        '''Z axis rotation'''
        if(axis[2]):
            x = x*math.cos(angle) - y*math.sin(angle)
            y = x*math.sin(angle) + y*math.cos(angle)
            #z  = z
            
            x_o = x_o*math.cos(angle) - y_o*math.sin(angle)
            y_o = x_o*math.sin(angle) + y_o*math.cos(angle)
            #z  = z
            
            
        '''Y axis rotation'''
        if(axis[1]):
            x = x*math.cos(angle) + z*math.sin(angle)
            #y = y
            z = -x*math.sin(angle) + z*math.cos(angle)
            
            x_o = x_o*math.cos(angle) + z_o*math.sin(angle)
            #y = y
            z_o = -x_o*math.sin(angle) + z_o*math.cos(angle)
            
            
        '''X axis rotation'''
        if(axis[0]):
            #x=x
            y = y*math.cos(angle) - z*math.sin(angle)
            z = y*math.sin(angle) + z*math.cos(angle)
            
            #x=x
            y_o = y_o*math.cos(angle) - z_o*math.sin(angle)
            z_o = y_o*math.sin(angle) + z_o*math.cos(angle)
            
            
        return Vector(x,y,z,x_o,y_o,z_o)
        
    def to_points(self):
        ''' Returns an array of [x,y,z] of the end points'''
        
        return [self.x,self.y,self.z]

    @classmethod
    def for_plotting(self,list_of_vectors):
        #init x,y,z points to start from the origin
        x_points = [0]
        y_points = [0]
        z_points = [0]
        for v in list_of_vectors:
            x_points.append(x_points[-1]+v[0])
            y_points.append(y_points[-1]+v[1])
            z_points.append(z_points[-1]+v[2])
        return (x_points,y_points,z_points)