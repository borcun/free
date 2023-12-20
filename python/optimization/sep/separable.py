import sys
from sympy import *

class Separable:
    """
    constructor
    """
    def __init__(self):
        self.objective = None
        self.constraints = []
        
    """
    function that gets equations from input file
    Params:
      path - input file path
    Return:
      True if file is read successfully, otherwise return False
    """
    def getEquations(self, path):
        self.objective = None
        self.constraints.clear()
        
        try:
            pFile = open(path, "r")
            line = pFile.readline()

            if line:
                # the objective function is assumed being first entry
                self.objective = sympify(line)
            
                while line:
                    line = pFile.readline()

                    if line:
                        self.constraints.append(sympify(line))

            pFile.close()

            if None == self.objective or 0 == len(self.constraints):
                print("The objective and constraint functions are missing")
                return False
            
            return True
        
        except:
            print("Could not read", path)

        return False

    """
    function that executes seprable algorithm
    Params:
      path - input file path
    Return:
      True if the algorithm executes successfully, otherwise return False
    """
    def execute(self, path):
        if not self.getEquations(path):
            return False

        x1, x2 = symbols('x1 x2')
        variables = [x1, x2]
        weight_list = [['w11', 'w12', 'w13', 'w14'],
                       ['w21', 'w22', 'w23', 'w24']]
        weights = [symbols(each) for each in weight_list]
        expressions = []
        is_x1_linear = True
        is_x2_linear = True
        
        # separate objective function
        f1x1 = self.objective.subs(x2, 0)
        f1x2 = self.objective.subs(x1, 0)

        if degree(f1x1) > 1:
            expressions.append(f1x1.subs(x1, 0) * weights[0][0] +
                               f1x1.subs(x1, 1) * weights[0][1] +
                               f1x1.subs(x1, 2) * weights[0][2] +
                               f1x1.subs(x1, 3) * weights[0][3])
            is_x1_linear = False
        else:
            expressions.append(f1x1)
            
        if degree(f1x2) > 1:
            expressions.append(f1x2.subs(x2, 0) * weights[1][0] +
                               f1x2.subs(x2, 1) * weights[1][1] +
                               f1x2.subs(x2, 2) * weights[1][2] +
                               f1x2.subs(x2, 3) * weights[1][3])
            is_x2_linear = False
        else:
            expressions.append(f1x2)

        # separate constraint functions
        gx1 = []
        gx2 = []

        for i in range(len(self.constraints)):
            gx1.append(self.constraints[i].subs(x2, 0).lhs)

            if degree(gx1[i]) > 1:
                expressions.append(gx1[i].subs(x1, 0) * weights[0][0] +
                                   gx1[i].subs(x1, 1) * weights[0][1] +
                                   gx1[i].subs(x1, 2) * weights[0][2] +
                                   gx1[i].subs(x1, 3) * weights[0][3])
                is_x1_linear = False
            else:
                expressions.append(gx1[i])

            gx2.append(self.constraints[i].subs(x1, 0).lhs)

            if degree(gx2[i]) > 1:
                expressions.append(gx2[i].subs(x2, 0) * weights[1][0] +
                                   gx2[i].subs(x2, 1) * weights[1][1] +
                                   gx2[i].subs(x2, 2) * weights[1][2] +
                                   gx2[i].subs(x2, 3) * weights[1][3])
                is_x2_linear = False
            else:
                expressions.append(gx2[i])                

        print("\nWeighted Functions")
        print("==================")
        print(" z =", self.objective.subs({f1x1: expressions[0], f1x2: expressions[1]}))

        for i in range(2, len(expressions) - 1, 2):
            print(" c" + str(i-1) + " =", self.constraints[i-2].subs({gx1[i-2]: expressions[i], gx2[i-2]: expressions[i+1]}))

        if not is_x1_linear:
            expressions.append(weights[0][0] + weights[0][1] + weights[0][2] + weights[0][3])
            print("", expressions[-1], "= 1")
                            
        if not is_x2_linear:
            expressions.append(weights[1][0] + weights[1][1] + weights[1][2] + weights[1][3])
            print("", expressions[-1], "= 1")
            
        return True
