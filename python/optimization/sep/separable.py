import sys
from sympy import *

class Separable:
    def __init__(self):
        self.equations = []
        self.fx = []
        self.gx = []

    """
    function that gets equations from input file
    """
    def getEquations(self, path):
        self.equations.clear()
        
        try:
            pFile = open(path, "r")
            line = pFile.readline()
            
            while line:
                self.equations.append(line)
                line = pFile.readline()

            pFile.close()

            return 0 != len(self.equations)
        
        except:
            print("Could not read", path)

        return False

    """
    """
    def execute(self, path):
        if not self.getEquations(path):
            return False

        x1, x2 = symbols('x1 x2')
        ind_vars = [x1, x2]
        expressions = []
        sublist = [['w11', 'w12', 'w13', 'w14'],
                   ['w21', 'w22', 'w23', 'w24']]
        
        sym_list = [symbols(each) for each in sublist]

        
        expr = sympify(self.equations[0])
        self.fx.append(expr.subs(x2, 0))
        self.fx.append(expr.subs(x1, 0))

        for i in range(1, len(self.equations)):
            expr = sympify(self.equations[i])
            tmp = []
            tmp.append(expr.subs(x2, 0))
            tmp.append(expr.subs(x1, 0))
            
            self.gx.append(tmp)
            

        for j in range(len(self.fx)):
            if degree(self.fx[j]) > 1:
                expressions.append(self.fx[j].subs(ind_vars[j], 0) * sym_list[j][0] +
                                   self.fx[j].subs(ind_vars[j], 1) * sym_list[j][1] +
                                   self.fx[j].subs(ind_vars[j], 2) * sym_list[j][2] +
                                   self.fx[j].subs(ind_vars[j], 3) * sym_list[j][3])
            else:
                expressions.append(None)

        
        for i in range(len(self.gx)):
            for j in range(len(self.gx[i])):
                if degree(self.gx[i][j]) > 1:
                    expressions.append(self.gx[i][j].subs(ind_vars[j], 0) * sym_list[j][0] +
                                       self.gx[i][j].subs(ind_vars[j], 1) * sym_list[j][1] +
                                       self.gx[i][j].subs(ind_vars[j], 2) * sym_list[j][2] +
                                       self.gx[i][j].subs(ind_vars[j], 3) * sym_list[j][3])
                else:
                    expressions.append(None)

        for i in range(len(expressions)):
            print(expressions[i])                
                    
                            
        return True
