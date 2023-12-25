"""
  The idea of direct search method is to identify the 'interval of uncertainty' known
to include the optimum solution point. The procedure locates the optimum by iteratively
narrowing the interval of uncertainty to a desired level of accuracy.
  Direct search methods apply primarily to strictly uni-modal single-variable functions.
Two closely related search algorithms are presented: dichotomous and golden section. These
two methods start with the initial interval of uncertainty I0 = (a, b)
"""

import sys
from sympy import *

class DirectSearch:
    """
    Default constructor
    """
    def __init__(self):
        # equation string that is set from file content
        self.objective = ""
        # first and second points supplied by user
        self.limit1 = 0
        self.limit2 = 0
        # accuracy value (aka delta in dichotomous approach) supplied by user
        self.accuracy = 0
        self.epsilon = 0


    """
    Function that gets equations for Direct Search algorithm

    Params:
      path - input file path
    Return:
      True if the equations are read from the file successfully, otherwise False
    """
    def getEquations(self, path):
        try:
            input_file = open(path, "r")
            self.objective = input_file.readline()
            input_file.close()
            
            return True
        except:
            print("Could not read", path)
            
        return False


    """
    Function that gets input parameters such as X, Y coordinate and iteration count
    """
    def getInputParameters(self):
        is_params_ok = False

        while not is_params_ok:
            try:
                self.limit1 = float(input(" Enter x of the first limit point  : "))
                self.limit2 = float(input(" Enter x of the second limit point : "))
                self.accuracy = float(input(" Enter accuracy level : "))
                self.epsilon = float(input(" Enter epsilon level  : "))

                if self.accuracy < 0 or self.epsilon < 0:
                    print("\n Please enter positive number for accuracy and epsilon\n")
                else:
                    is_params_ok = True
                
            except ValueError:
                print("\n Could not convert entries to float numbers\n")


    """
    Function that executes Direct Search algorithm

    Params:
      path - input file path
    Return:
      True if the algorithm is executed successfully, otherwise False
    """
    def execute(self, path):
        if not self.getEquations(path):
            return False
        
        self.getInputParameters()

        x = symbols('x')
        expr = sympify(self.objective)
        is_found = False
        count = 1

        while not is_found:
            point1 = 0.5 * (self.limit1 + self.limit2 - self.epsilon)
            point2 = 0.5 * (self.limit1 + self.limit2 + self.epsilon)

            y1 = expr.subs(x, point1)
            y2 = expr.subs(x, point2)

            print("\n--- Step {0} -----------------------------------------------\n".format(count))
            print(" The limit values: ", self.limit1, ",", self.limit2)
            print(" The points: ", point1, ", ", point2)
            print(" f(x1) =", y1, ", f(x2) =", y2)

            if y1 > y2:
                self.limit1 = point1
            elif y1 < y2:
                self.limit2 = point2
            else:
                self.limit1 = point1
                self.limit2 = point2
              
            count += 1

            # flag to terminate iteration
            if abs(y1 - y2) <= self.accuracy:
                print("\n The final limit values:", point1, ", ", point2)
                print(" f({0}) = {1}\n f({2}) = {3}".format(point1, expr.subs(x, point1), point2, expr.subs(x, point2)))
                is_found = True

        return True
