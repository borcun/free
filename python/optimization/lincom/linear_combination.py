from sympy import *
from scipy.optimize import linprog

"""
LinearCombinations Method class
"""
class LinearCombinations:
    """
    Default constructor
    """
    def __init__(self):
        self.objective = None
        self.constraints = []
        # x and y is starting point supplied by user
        self.x = 0
        self.y = 0
        # iteration count supplied by user
        self.count = 0

    """
    Function that gets equations from input file
    
    Params:
      path - input file path
    Return:
      True if file is read successfully, otherwise return False
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
    Function that gets input parameters such as X, Y coordinate and iteration count
    """
    def getInputParameters(self):
        is_params_ok = False

        while not is_params_ok:
            try:
                self.x = float(input(" Enter x of start point : "))
                self.y = float(input(" Enter y of start point : "))
                self.count = int(input(" Enter iteration count : "))

                if self.count < 1:
                    print("\n Please enter positive number for iteration count\n")
                else:
                    is_params_ok = True

            except ValueError:
                print("\n Could not convert entries to numbers, enter again please\n")

    """
    Function that executes the algorithm
    
    Params:
      path - path of the input file including equation
    Return:
      True if execution is completed, otherwise False
    """
    def execute(self, path):
        if not self.getEquations(path):
            return False

        self.getInputParameters()

        x1, x2, r = symbols('x1 x2 r')
        expr = sympify(self.objective)
        delta_x1 = None
        delta_x2 = None
        x = self.x
        y = self.y

        dfx1 = diff(expr, x1)
        dfx2 = diff(expr, x2)
        A = [list(constraint.lhs.as_coefficients_dict().values()) for constraint in self.constraints]
        b = [constraint.rhs for constraint in self.constraints]

        print("-----------------------------------------------------")
        print("---------- The function and its derivation ----------")
        print("-----------------------------------------------------")
        print("f(x1, x2) =", expr)
        print("f(x1, x2)' = ({0}, {1})\n".format(dfx1, dfx2))

        for i in range(self.count):
            if delta_x1 == dfx1.subs({x1: x, x2: y}) and delta_x2 == dfx2.subs({x1: x, x2: y}):
                print(" Note: The result is same with previous one, so no need more iteration!\n")
                break

            print("--- Step {0} -----------------------------------------------\n".format(i + 1))
            print(" x{0} = ({1}, {2})".format(i, x, y))

            delta_x1 = dfx1.subs({x1: x, x2: y})
            delta_x2 = dfx2.subs({x1: x, x2: y})

            w = linprog([-delta_x1, -delta_x2], A_ub=A, b_ub=b, bounds=[(0, None), (0, None)])
            hr = None

            print(" f(x{0})' = ({1}, {2})".format(i, delta_x1, delta_x2))

            # TODO I am not thoroughly sure for statement within the else case, will check !!!
            if abs(w.fun) > delta_x1 * x + delta_x2 * y:
                hr = expr.subs({x1: x + r * (w.x[0] - x), x2: y + r * (w.x[1] - y)})
            else:
                hr = expr.subs({x1: delta_x1 * r + x, x2: delta_x2 * r + x})

            print(" h(r) =", hr)
            r1 = solve(diff(hr, r))

            print(" r1 = {0}\n".format(r1))
            x = x + (r1[0] * (w.x[0] - x))
            y = y + (r1[0] * (w.x[1] - y))

        print("--- Result ------------------------------------------")
        print("The maximization point (x, y) is ({0}, {1})".format(x, y))
        print("Maximization of f(x1, x2) = ", expr.subs({x1: x, x2: y}))
        print("-----------------------------------------------------\n")

        return True
