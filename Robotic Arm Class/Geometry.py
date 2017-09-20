# -*- coding: utf-8 -*-
"""
Created on Tue Sep 19 19:52:49 2017

@author: joell
"""
import math
import numbers

class Point(object):
    '''
    represents a point in 3D space in cartisian coordinates.

    Parameters
    ----------
        x : scalar or arraylike
            this is either the x coordinate that the point will represent or a list that conatins
            the [x,y,z] coordindates
        
        y,z : scalars (optional)
            these are the y,z coordinates that the point will represent

    Returns
    -------
        None
    '''
    x = 0
    y = 0
    z = 0

    def __init__(self, x_car, y_car=0.0, z_car=0.0):
        if isinstance(x_car, numbers.Number):
            self.x = float(x_car)
            self.y = float(y_car)
            self.z = float(z_car)
        elif isinstance(x_car, list):
            if len(x_car) != 3:
                raise IndexError("Wrong length of array. Need [x,y,z] coordinates for a Point")
            self.x = float(x_car[0])
            self.y = float(x_car[1])
            self.z = float(x_car[2])

    def __repr__(self):
        '''
        This the string representation of the object.

        Parameters
        ----------
            None

        Returns
        -------
            String containing the type of class and x,y,z coordinates
        '''

        return "{}({:.2f},{:.2f},{:.2f})".format(self.__class__.__name__,
                                                 self.x,
                                                 self.y,
                                                 self.z)

    def __str__(self):
        '''
        This is the string version of the object contents. This is what is called if you print
        an object.

        Parameters
        ----------
            None

        Returns
        -------
            String containing the x,y,z coordinates seperated by commas
        '''
        return "{:.3f},{:.3f},{:.3f}".format(self.x, self.y, self.z)

    def __add__(self, other_point):
        '''
        This is the add method that occurs if you have Point + object.

        Parameters
        ----------
            other_point : Point
                something that you want to have its contents added with this object.

        Returns
        -------
            out : Point
                A Point object that is the sum of the current point and the other point

        Raises
        ------
            TypeError
                If the object that you are trying to add is not a Point object
        '''

        if isinstance(other_point, Point):
            return Point(self.x + other_point.x, self.y+other_point.y, self.z + other_point.z)
        else:
            raise TypeError("Tried to add a non-point thing with a point.")

    def __sub__(self, other_point):
        '''
        This is the subtract method that occurs if you have Point - object.

        Parameters
        ----------
            other_point : Point
                something that you want to have its contents subtracted with this object.

        Returns
        -------
            out : Point
                 A Point object that is the subtraction of the current point and the other point

        Raises
        ------
            TypeError
                If the object that you are trying to subtract is not a Point object
        '''
        if isinstance(other_point, Point):
            return Point(self.x - other_point.x, self.y - other_point.y, self.z - other_point.z)
        else:
            raise TypeError("Tried to subtract a non-point thing with a point.")

    def __getitem__(self, index):
        '''
        This is the if you use the bracket notation to get at the contence of the object.

        Parameters
        ----------
            index : Integer or String
                Valid options are 0,1,2,x,y,z

        Returns
        -------
            out : scalar
                 value of the thing you want

        Raises
        ------
            IndexError
                If the index is not on the list of indicies
        '''

        if index in [0, 1, 2, 'x', 'y', 'z']:
            if index == 0:
                return self.x
            elif index == 1:
                return self.y
            elif index == 2:
                return self.z
            elif index == 'x':
                return self.x
            elif index == 'y':
                return self.y
            else:
                return self.z
        else:
            raise IndexError("Incorrect index. Valid index options are [1,2,3,'x','y','z']")

    def __mul__(self, another_point):
        '''
        This is used for preforming the 'dot product'. Normal use wouldnt make much sense.

        Parameters
        ----------
            another_point : scalar or Point
                If a scalar is given the point will be scaled in the x,y,z axis by that amount.
                If a point is given it preforms element wise multiplication

        Returns
        -------
            out : Point
                A Point representing each element multiplied

        '''
        if isinstance(another_point, numbers.Number):
            return Point(
                self.x*another_point,
                self.y*another_point,
                self.z*another_point)
        else:
            return Point(
                self.x*another_point.x,
                self.y*another_point.y,
                self.z*another_point.z)
                
    def __div__(self, another_point):
        '''
        Element wise division

        Parameters
        ----------
            another_point : scalar or Point
                If a scalar is given the point will be scaled in the x,y,z axis by that amount.
                If a point is given it preforms element wise division

        Returns
        -------
            out : Point
                A Point representing each element divided

        '''
        if isinstance(another_point, numbers.Number):
            return Point(
                self.x/another_point,
                self.y/another_point,
                self.z/another_point)
        else:
            return Point(
                self.x/another_point.x,
                self.y/another_point.y,
                self.z/another_point.z)

    def as_array(self):
        '''
        Returns the x,y,z coordinates of the point in an array.

        Parameters
        ----------
            None

        Returns
        -------
            out : arraylike
                an array that has x, y, and z in that order in it
        '''

        return [self.x, self.y, self.z]
    
    def dot(self, point):
        """
        Return the dot product of two point.

        a.dot(b) = a1*b1+a2*b2+a3*b3

        Parameters
        ----------
            point : Point
                A vector to dot with the current object

        Returns:
            a number representing the dot product of vector passed in and the
            current vector.


        """
        return self.x*point.x+self.y*point.y+self.z*point.z

class Vector(object):
    """
        Vector class:
            Representing a vector in 3D space.

        Can accept formats of:
            Cartesian coordinates in the x, y, z space.
            Spherical coordinates in the r, theta, phi space.
            Cylindrical coordinates in the r, theta, z space.


        Parameters
        ----------
            args:
                Can accept 0,1,2,3, or 6 arguments defining the vector.
                The first 3 arguments must define the ending point of the vector.

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

        Returns
        -------
            The A vector object going from the origin to the point specied

        Examples
        --------

        >>> Vector.Vector()      #creates a vector going from (0,0,0) -> (0,0,0)
        >>> Vector.Vector(1)     #creates a vector going from (0,0,0) -> (1,0,0)
        >>> Vector.Vector(1,2)   #creates a vector going from (0,0,0) -> (1,2,0)
        >>> Vector.Vector(1,2,3) #creates a vector going from (0,0,0) -> (1,2,3)
        >>> Vector.Vector(1,2,3,5,4,3) #creates a vector going from (5,4,3) -> (1,2,3)
        >>> Vector.Vector(1,1,1,cylindrical=True)
        #creates a vector going from (0,0,0) -> (0.540302305868, 0.841470984808, 1)
    """

    def __init__(self, *args, **kwargs):
        self.origin = Point(0.0, 0.0, 0.0)
        self.end = Point(0.0, 0.0, 0.0)

        if len(args) == 1:
            if isinstance(args[0], Point):
                self.end = args[0]
            else:
                self.end.x = float(args[0])
        elif len(args) == 2:
            if isinstance(args[0], Point) and isinstance(args[1], Point):
                self.end = args[0]
                self.origin = args[1]
            else:
                self.end.x = float(args[0])
                self.end.y = float(args[1])

        elif len(args) == 3 or len(args) == 6:
            if kwargs.has_key('spherical'):
                if kwargs['spherical'] != False:
                    self.end.x = args[0]*math.sin(args[1])*math.cos(args[2])
                    self.end.y = args[0]*math.sin(args[1])*math.sin(args[2])
                    self.end.z = args[0]*math.cos(args[1])
            elif kwargs.has_key('cylindrical'):
                if kwargs['cylindrical'] != False:
                    self.end.x = args[0]*math.cos(args[1])
                    self.end.y = args[0]*math.sin(args[1])
                    self.end.z = float(args[2])
            else:
                self.end.x = float(args[0])
                self.end.y = float(args[1])
                self.end.z = float(args[2])

            if len(args) == 6:
                self.origin.x = float(args[3])
                self.origin.y = float(args[4])
                self.origin.z = float(args[5])
        elif len(args) == 4:
            self.end.x = float(args[0])
            self.end.y = float(args[1])
            self.end.z = float(args[2])
            self.origin = args[3]


    def __add__(self, another_vector):
        """
        Adding two vectors together.

        This is done in tip to tail method. If either (or both) vectors are not
        from the origin then it moves them to the orign. If the vector being based
        in is scalar (ie. type of float, int, or long) then the value will be added
        to x, y, and z onto the end; maintaining the origin if it is non-zero.


        """
        if isinstance(another_vector, Vector):
            return Vector(self.end + another_vector.end, self.origin + another_vector.origin)

        elif isinstance(another_vector, numbers.Number):
            return Vector(self.end.x + another_vector,
                          self.end.y + another_vector,
                          self.end.z + another_vector,
                          self.origin)
        else:
            raise TypeError("Tried to add a {} type to a vector. Requres a vector object, int, float, or long".format(type(another_vector)))

    def __sub__(self, another_vector):
        """
        subtracting two vectors .

        This is done in tip to tail method. If either (or both) vectors are not
        from the origin then it moves them to the orign. If the vector being based
        in is scalar (ie. type of float, int, or long) then the value will be subtracted
        to x, y, and z onto the end; maintaining the origin if it is non-zero.


        """
        if isinstance(another_vector, Vector):
            return Vector(self.end - another_vector.end, self.origin - another_vector.origin)

        elif isinstance(another_vector, numbers.Number):
            return Vector(self.end.x - another_vector,
                          self.end.y - another_vector,
                          self.end.z - another_vector,
                          self.origin)
        else:
            raise TypeError("Tried to subtract a {} type to a vector. Requres a vector object, int, float, or long".format(type(another_vector)))

    def __mul__(self, another_vector):
        """ Return a Vector instance as the cross product of two vectors """
        return self.cross(another_vector)
    
    def __div__(self, a_scalar):
        '''
        Returns the element wise division of the vector
        '''
        if isinstance(a_scalar, numbers.Number):
            if a_scalar != 0:
                return Vector(self.end*1.0/a_scalar,self.origin*1.0/a_scalar)
            else:
                raise ValueError("Tried to divid a vector by zero")
        else:
            raise TypeError("Vector division is only supported with scalar numbers")

    def __str__(self):
        """
        Returns the string representation of the vector.

        This is the one that is used by print

        The format that is set is (x_origin,y_origin,z_origin) -> (x,y,z)

        >>> v1 = Vector(1,2,3)
        >>> print v1
        >>> '(0,0,0) -> (1,2,3)'
        """
        return "({1}{0}{2}{0}{3}) -> ({4}{0}{5}{0}{6})".format(",",
                                                               self.origin.x,
                                                               self.origin.y,
                                                               self.origin.z,
                                                               self.end.x,
                                                               self.end.y,
                                                               self.end.z)

    def __repr__(self):
        """
        Returns the string representation of the vector with the class name.


        The format that is set is 'Vector object: (x_origin,y_origin,z_origin) -> (x,y,z)'

        >>> v1 = Vector(1,2,3)
        >>> print repr(v1)
        >>> 'Vector Object: (0,0,0) -> (1,2,3)'
        """
        return "Vector object: ({1}{0}{2}{0}{3}) -> ({4}{0}{5}{0}{6})".format(",",
                                                                              self.origin.x,
                                                                              self.origin.y,
                                                                              self.origin.z,
                                                                              self.end.x,
                                                                              self.end.y,
                                                                              self.end.z)

    def __getitem__(self, index):
        """
        Allows for index notation in the vector.

        Returns:
            the x,y, or z component of the vector. This IS from the modified origin

        >>> v1 = Vector(1,2,3)
        >>> print v1[1]
        >>> '2'

        """
        if index in [0, 1, 2, 3, 4, 5]:
            if index == 0:
                return self.end.x
            elif index == 1:
                return self.end.y
            elif index == 2:
                return self.end.z
            elif index == 3:
                return self.origin.x
            elif index == 4:
                return self.origin.y
            else:
                return self.origin.z
        else:
            raise IndexError("Invalid index. Valid index's are [0-5]")

    @staticmethod
    def magnitude(vector):
        '''
        This computes the magnitude to the input vector. A magnitude is all elements squared,
        summed, and then square rooted.

        Parameters
        ----------
            vector : arraylike or Vector
                Array that will have the magnitude computed. It must be a 1D array of size n.

        Returns
        -------
            out : scalar
                the vector length

        More Information
        ----------------
            For more information on how this could be used in a more general sense please see the
            link. https://en.wikipedia.org/wiki/Euclidean_vector
        '''
        out = 0
        if isinstance(vector, Vector):
            for temp_end, temp_origin in zip(vector.end.as_array(), vector.origin.as_array()):
                out += math.pow(temp_end-temp_origin, 2)
        elif isinstance(vector, Point):
            for counter in vector.as_array():
                out += math.pow(counter, 2)
        else:
            for counter in vector:
                out += counter*counter
        return math.sqrt(out)

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

        Parameters
        ----------
            vector : Vector
                A vector to dot with the current object

        Returns:
            a number representing the dot product of vector passed in and the
            current vector.


        """
        adjusted_point = self.end - self.origin
        adjusted_vector = vector.end - vector.origin
        multiplied_point = adjusted_point*adjusted_vector
        return sum(multiplied_point.as_array())

    def cross(self, vector):
        """
        Return a Vector instance as the cross product of two vectors

        """

        assert type(vector) is type(self), "Tried to take the cross product of %s type object, only vectors"%(str(type(vector)))
        adjusted_vector_1 = self.end - self.origin
        adjusted_vector_2 = vector.end - vector.origin
        return Vector(
            (adjusted_vector_1.y * adjusted_vector_2.z - adjusted_vector_1.z * adjusted_vector_2.y),
            (adjusted_vector_1.z * adjusted_vector_2.x - adjusted_vector_1.x * adjusted_vector_2.z),
            (adjusted_vector_1.x * adjusted_vector_2.y - adjusted_vector_1.y * adjusted_vector_2.x))

    def angle(self, vector):
        """
        Return the angle between two vectors in radians.

        theta = acos((u(dot)v)/(||u||*||v||))

        u (dot) v is the dot product between u and v

        ||u|| is the length (magnitude) of the vector
        """
        top_fraction = self.dot(vector)
        bot_fraction = Vector.magnitude(self)*Vector.magnitude(vector)
        return math.acos(top_fraction/bot_fraction)

    def parallel(self, vector):
        """ Return True if vectors are parallel to each other. """

        return True if Vector.magnitude(self.cross(vector)) == 0 else False

    def perpendicular(self, vector):
        """ Return True if vectors are perpendicular to each other. """

        return True if self.dot(vector) == 0 else False



    def rotate(self, angle, axis=(0, 0, 1)):
        """
        Angle is the amount of rotation in radians.

        Axis is a tuple or array indicating which axis to rotate around. (x,y,z).
        If more than 1 axis is specified then it will rotate around in the order z,y,x.

        Returns a rotated vector with rotated origin.

        """

        if isinstance(axis[0], int) and isinstance(axis[1], int) and isinstance(axis[2], int):
            raise ValueError("cannot rotate on a non integer axis, must be in the format of (int,int,int)")

        ending_point = Point(self.end.x, self.end.y, self.end.z)
        starting_point = Point(self.origin.x, self.origin.y, self.origin.z)

        # Z axis rotation
        if axis[2]:
            ending_point.x = ending_point.x*math.cos(angle) - ending_point.y*math.sin(angle)
            ending_point.y = ending_point.x*math.sin(angle) + ending_point.y*math.cos(angle)

            starting_point.x = starting_point.x*math.cos(angle) - starting_point.y*math.sin(angle)
            starting_point.y = starting_point.x*math.sin(angle) + starting_point.y*math.cos(angle)


        # Y axis rotation
        if axis[1]:
            ending_point.x = ending_point.x*math.cos(angle) + ending_point.z*math.sin(angle)
            ending_point.z = -ending_point.x*math.sin(angle) + ending_point.z*math.cos(angle)

            starting_point.x = starting_point.x*math.cos(angle) + starting_point.z*math.sin(angle)
            starting_point.z = -starting_point.x*math.sin(angle) + starting_point.z*math.cos(angle)


        # X axis rotation
        if axis[0]:
            ending_point.y = ending_point.y*math.cos(angle) - ending_point.z*math.sin(angle)
            ending_point.z = ending_point.y*math.sin(angle) + ending_point.z*math.cos(angle)

            starting_point.y = starting_point.y*math.cos(angle) - starting_point.z*math.sin(angle)
            starting_point.z = starting_point.y*math.sin(angle) + starting_point.z*math.cos(angle)


        return Vector(ending_point, starting_point)

    def to_points(self):
        ''' Returns an array of [x,y,z] of the end points'''

        return self.end.as_array()

    @classmethod
    def for_plotting(cls, list_of_vectors):
        '''
        This is used for plotting vectors.

        Parameters
        ----------
            list_of_vetors : arraylike
                an array or list containg vectors that you want formated.

        Returns
        -------
            out : arraylike
                Array containing all of the x coordinates, y coordinates, and z coordinates

        Note
        ----
            This assumes that all the vectors have their origin at 0,0,0. If the vector doesnt
            have its origin at zero, then it will appear to "grow" or "change direction"
        '''

        #init x,y,z points to start from the origin
        x_points = [0]
        y_points = [0]
        z_points = [0]
        for temp_vector in list_of_vectors:
            x_points.append(x_points[-1]+temp_vector[0])
            y_points.append(y_points[-1]+temp_vector[1])
            z_points.append(z_points[-1]+temp_vector[2])
        return [x_points, y_points, z_points]
    
    @staticmethod
    def normalize(vector):
        '''
        creates the vector that is pointint in the same direction as the input vector but with a
        length of 1.

        Parameters
        ----------
            vector : Vector
                input vector that will become a unit vector.

        Returns
        -------
            out : Vector
                unit vector pointing in the same direction as the input vector but from the origin
        '''
        length = Vector.magnitude(vector)
        delta_end = vector.end-vector.origin
        return Vector(delta_end.x/length, delta_end.y/length, delta_end.z/length)
