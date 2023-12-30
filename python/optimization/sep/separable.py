"""
The linear equation existed after linear approximation are known as a linear approximation of the original NLP,
and can be solved heuristically by a variant of the simplex algorithm for LP.
"""

from sympy import *
import algorithm

class Separable(alg.Algorithm):
    def __init__(self):
        self.objective = None
        self.constraints = []

    def read(self, path):
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

    def solve(self):
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
            
        print("")
        self.printTable(expressions)

        return True

    def printTable(self, expressions):
        x1, x2 = symbols('x1 x2')
        weight_list = [['w11', 'w12', 'w13', 'w14'],
                       ['w21', 'w22', 'w23', 'w24']]
        
        obj = self.objective.subs({self.objective.subs(x2, 0): expressions[0], self.objective.subs(x1, 0): expressions[1]})
        obj_syms = list(obj.atoms(Symbol))
        obj_coeffs = [-1 * obj.coeff(elem) for elem in obj_syms]

        print("----------------------------------------------------------------")
        print("Basic\t", end = '')

        for i in range(len(obj_syms)):
            print("{0}\t".format(obj_syms[i]), end = '')

        for i in range(len(self.constraints)):
            print("s{0}\t".format(i + 1), end = '')

        for i in range(len(self.constraints)):
            print("w2{0}\t".format(i + 1), end = '')

        print("Solution")
        print("----------------------------------------------------------------")
        print("z\t", end = '')

        for i in range(len(obj_coeffs)):
            print("{0}\t".format(obj_coeffs[i]), end = '')

        for i in range(len(self.constraints) + 1):
            print("{0}\t".format(0), end = '')

        print("0")
        print("----------------------------------------------------------------")

        for i in range(len(self.constraints)):
            constraint = self.constraints[i].lhs.subs({self.constraints[i].lhs.subs(x2, 0): expressions[i + 2], self.constraints[i].lhs.subs(x1, 0): expressions[i + 3]})
            constraint_syms = list(constraint.atoms(Symbol))
            constraint_coeffs = [constraint.coeff(elem) for elem in constraint_syms]
          
            print("s{0}\t ".format(i + 1), end = '')

            for j in range(len(constraint_syms)):
                print("{0}\t".format(constraint_coeffs[j]), end = '')

            for j in range(len(self.constraints) + 1):
                if i == j:
                    print("{0}\t".format(1), end = '')
                else:
                    print("{0}\t".format(0), end = '')

            print(self.constraints[i].rhs)

        bi = 0
        basis_condition_count = len(expressions) - (2 + 2 * len(self.constraints))

        if basis_condition_count > 0:
            for i in range(2 + 2 * len(self.constraints), len(expressions), 1):
                bc_syms = list(expressions[i].atoms(Symbol))
                bc_coeffs = [expressions[i].coeff(elem) for elem in bc_syms]

                print("w2{0}\t 0\t".format(bi + 1), end = '')

                for j in range(len(bc_syms) - 1):
                    print("{0}\t".format(bc_coeffs[j]), end = '')

                print("0\t1\t1")

        print("----------------------------------------------------------------")
