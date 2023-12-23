"""
  The steepest-ascent method is a method for unconstrained mathematical optimization. It
is used for optimizing twice continuously differentiable function. The idea is to generate
successive points in the direction of the gradient of the function. Termination of the
gradient method occurs at the point where the gradient vector becomes null. This is only
a necessary condition for optimality.
"""

import sys
from sympy import *

"""
Gradient Method class
"""
class Gradient:
    """
    Constructor
    """
    def __init__(self):
        # equation string that is set from file content
        self.equation = ""
        # x and y is starting point supplied by user
        self.x = 0
        self.y = 0
        # iteration count supplied by user
        self.count = 0

    """
    Function that gets equation for Gradient Method algorithm
    Params:
      path - input file path
    Return:
      True if the file is read, otherwise False
    """
    def getEquation(self, path):
        try:
            pFile = open(path, "r")
            self.equation = pFile.readline()
            pFile.close()
            
            return True
        except:
            print("Could not read", path)
        
        return False

    """
    Function that gets input parameters such as X, Y coordinate and iteration count
    """
    def getInputParameters(self):
        while True:
            try:
                print(" Enter x of start point : ", end = '')
                self.x = float(input())
                print(" Enter y of start point : ", end = '')
                self.y = float(input())
                print(" Enter iteration count  : ", end = '')
                self.count = int(input())

                if self.count < 1:
                    print("\n Please enter positive number for iteration count\n")
                else:
                    break
                
            except:
                print("\n Could not convert entry to integer number\n")
                pass       
            

    """
    Function that executes the algorithm
    Params:
      path - pat of the input file including equation
    Return:
      True if execution is completed, otherwise False
    """
    def execute(self, path):
        if not self.getEquation(path):
            return False

        self.getInputParameters()
        
        x1, x2, r = symbols('x1 x2 r')
        expr = sympify(self.equation)
        dfx1, dfx2 = diff(expr, x1), diff(expr, x2)

        # selected points
        x, y = self.x, self.y

        print("-----------------------------------------------------")
        print("---------- The function and its derivation ----------")
        print("-----------------------------------------------------")
        print("f(x1, x2) =", expr)
        print("f(x1, x2)' = ({0}, {1})\n".format(dfx1, dfx2))

        for i in range(self.count):
            print("--- Step {0} -----------------------------------------------\n".format(i + 1))
            print(" x{0} = ({1}, {2})".format(i, x, y))

            delta_x1 = dfx1.subs({x1: x, x2: y})
            delta_x2 = dfx2.subs({x1: x, x2: y})
        
            print(" f(x{0})' = ({1}, {2})".format(i, delta_x1, delta_x2))
            hr = expr.subs({x1: delta_x1 * r + x, x2: delta_x2 * r + y})
        
            print(" h(r) =", hr)
            r1 = solve(diff(hr, r))

            print(" r1 = {0}\n".format(r1))
            x = x + delta_x1 * r1[0].evalf()
            y = y + delta_x2 * r1[0].evalf()

        print("--- Result ------------------------------------------")
        print("The maximization point (x, y) is ({0}, {1})".format(x,  y))
        print("Maximization of f(x1, x2) = ", expr.subs({x1: x, x2: y}))
        print("-----------------------------------------------------\n")
        
        return True
