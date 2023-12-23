"""
  The idea of direct search method is to identify the 'interval of uncertainty' known
to include the optimum solution point. The procedure locates the optimum by iteratively
narrowing the interval of uncertainty to a desired level of accuracy.
  Direct search methods apply primarily to strictly unimodal single-variable functions.
Two closely related search algorithms are presented: dichotomous and golden section. These
two methods start with the initial interval of uncertainty I0 = (a, b)
"""

import sys
from sympy import *

class DirectSearch:
    def __init__(self):
        # equation string that is set from file content
        self.equation = ""
        # first and second points supplied by user
        self.limit1 = 0
        self.limit2 = 0
        # epsilon value (aka delta in dichotomous approach) supplied by user
        self.epsilon = 0

    """
    Function that gets equation for Direct Search Method algorithm
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
                print(" Enter x of the first limit point : ", end = '')
                self.limit1 = float(input())
                print(" Enter x of the second limit point : ", end = '')
                self.limit2 = float(input())
                print(" Enter epsilon value  : ", end = '')
                self.epsilon = float(input())

                if self.epsilon < 0:
                    print("\n Please enter positive number for epsilon\n")
                else:
                    break
                
            except:
                print("\n Could not convert entries to float numbers\n")
                pass

    def execute(self, path):
        if not self.getEquation(path):
            return False
        
        self.getInputParameters()

        x = symbols('x')
        expr = sympify(self.equation)
        isIterationDone = False
        iter = 1

        while not isIterationDone:
          point1 = self.limit1 + 0.5 * abs(self.limit1 - self.limit2 - self.epsilon)
          point2 = self.limit1 + 0.5 * abs(self.limit1 - self.limit2 + self.epsilon)

          y1 = expr.subs(x, point1)
          y2 = expr.subs(x, point2)

          print("\n--- Step {0} -----------------------------------------------\n".format(iter))
          print(" The limit values: ", self.limit1, ",", self.limit2)
          print(" The points: ", point1, ", ", point2)
          print(" f(x1) =", y1, ", f(x2) =", y2)

          if y1 > y2:
            self.limit2 = point2
          elif y1 < y2:
            self.limit1 = point1
          else:
            self.limit1 = point1
            self.limit2 = point2
          
          iter += 1

          # flag to terminate iteration
          if abs(self.limit1 - self.limit2) <= self.epsilon:
            isIterationDone = True

        print("\n The final limit values:", self.limit1, ", ", self.limit2)
        print(" f({0}) = {1}\n f({2}) = {3}".format(self.limit1, expr.subs(x, self.limit1), self.limit2, expr.subs(x, self.limit2)))
    return True
