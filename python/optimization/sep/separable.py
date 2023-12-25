from sympy import *

class Separable:
    """
    constructor
    """
    def __init__(self):
        self.objective = None
        self.constraints = []


    """
    Function that gets equations for Separable algorithm
    
    Params:
      path - input file path
    Return:
      True if the equations are read from the file successfully, otherwise False
    """
    def getEquations(self, path):
        self.objective = None
        self.constraints.clear()
        
        try:
            input_file = open(path, "r")
            line = input_file.readline()

            if line:
                # the objective function is assumed being first entry
                self.objective = sympify(line)
            
                while line:
                    line = input_file.readline()

                    if line:
                        self.constraints.append(sympify(line))

            input_file.close()

            if self.objective is None or 0 == len(self.constraints):
                print("The objective and constraint functions are missing")
                return False
            
            return True
        
        except:
            print("Could not read", path)

        return False

    """
    Function that executes the Separable algorithm

    Params:
      path - path of the input file including equation
    Return:
      algorithm result if the algorithm is executed successfully, otherwise False
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

        ci = 0
        
        for i in range(2, len(expressions), 2):
            print("", self.constraints[ci].subs({gx1[ci]: expressions[i], gx2[ci]: expressions[i+1]}))
            ci += 1

        if not is_x1_linear:
            expressions.append(weights[0][0] + weights[0][1] + weights[0][2] + weights[0][3])
            print("", expressions[-1], "= 1")
                            
        if not is_x2_linear:
            expressions.append(weights[1][0] + weights[1][1] + weights[1][2] + weights[1][3])
            print("", expressions[-1], "= 1")
            
        return True
