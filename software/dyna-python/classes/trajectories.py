import scipy.interpolate as interp
import numpy as np
import math


class trajectory:
    """
    The trajectory class is used to generate smooth unit transitions
    following certain selectable paths
    """
    
    def __init__(self, ttype, realtime, parameters=1, points=1000):
        """
        The object can either calculate the points in real time or pre-calculate
        and store them for table referen
        :param ttype: type of curve to use. Currently can choose between
                      a fifth order polinomial and a ciclical curve
        :param realtime: boolean to select the real-time or pre-calculated methods
        :param parameters: TODO: either use this or delete this
        :param points: number of points to calculate in case of the pre-calculated mode
        """
        self.ttype = ttype
        self.realtime = realtime
        self.parameters = parameters
        
        if ttype == 'p5':
            self.funct_rt = p5_rt
        elif ttype == 'cicle':
            self.funct_rt = cicle_rt
        elif ttype == 'cuad':
            self.funct_rt = cuad_rt
        else:
            print('Not an available trajectory')

        if not realtime:
            valuesy = []
            valuesx = []
            for i in range(points):
                valuesy.append(self.funct_rt(i/points))
                valuesx.append(i/points) 
            self.funct_interp = interp.interp1d(valuesx, valuesy)

    
    def get_value(self, x):
        """
        get_value returns the corresponding curve value based on the x input
        :param x: value from 0 to 1 to serve as reference
        :return y: value from curve
        """
        if self.realtime:
            y = self.funct_rt(x)
        else:
            # Interpolating between the two closest table values to get an
            # approximate result of the curve
            y = self.funct_interp(x)
            y = y.astype(np.float)
        return y
    

def p5_rt(x):
    """
    p5_rt returns the curve value following a fifth order polinomial
    :param x: x value to get curve result (goes from 0 to 1)
    :return y: curve result
    """
    y = 10*(x**3) - 15*(x**4) + 6*(x**5)
    return y


def cicle_rt(x):
    """
    cicle_rt returns the curve value following a ciclical curve
    :param x: x value to get curve result (goes from 0 to 1)
    :return y: curve result
    """
    y = x - (1/(2*math.pi))*math.sin(2*math.pi*x)
    return y

def cuad_rt(x):
    """
    cuad_rt returns the curve value following a square curve
    :param x: x value to get curve result (goes from 0 to 1)
    :return y: curve result
    """
    y = x**1.75
    return y
