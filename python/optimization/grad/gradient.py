"""
  The steepest-ascent method is a method for unconstrained mathematical optimization. It
is used for optimizing twice continuously differentiable function. The idea is to generate
successive points in the direction of the gradient of the function. Termination of the
gradient method occurs at the point where the gradient vector becomes null. This is only
a necessary condition for optimality.
"""

from sympy import *
import algorithm as alg

"""
Gradient Method class
"""
class Gradient(alg.Algorithm):
    def __init__(self):
        super().__init__()

        # equation string that is set from file content
        self.equation = ""
        # x and y is starting point supplied by user
        self.x = 0
        self.y = 0
        # iteration count supplied by user
        self.count = 0


    def read(self, path):
        try:
            input_file = open(path, "r")
            self.equation = input_file.readline()
            input_file.close()
            
            return True
        except:
            print("Could not read", path)
        
        return False


    def interact(self):
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


    def solve(self):
        x1, x2, r = symbols('x1 x2 r')
        expr = sympify(self.equation)
        dfx1, dfx2 = diff(expr, x1), diff(expr, x2)

        # selected points
        x, y = self.x, self.y

        print("-----------------------------------------------------")
        print("---------- The function and its derivation ----------")
        print("-----------------------------------------------------")
        print(" f(x1, x2) =", expr)
        print(" f(x1, x2)' = ({0}, {1})\n".format(dfx1, dfx2))

        for i in range(self.count):
            print("--- Step {0} -----------------------------------------------\n".format(i + 1))
            print(" x{0} = ({1}, {2})".format(i, x, y))

            delta_x1 = dfx1.subs({x1: x, x2: y})
            delta_x2 = dfx2.subs({x1: x, x2: y})
        
            print(" f(x{0})' = ({1}, {2})".format(i, delta_x1, delta_x2))
            hr = expr.subs({x1: delta_x1 * r + x, x2: delta_x2 * r + y})
        
            print(" h(r) =", hr)
            r1 = solve(diff(hr, r))

            if 0 < len(r1):
                print(" r1 = {0}\n".format(r1))
                x = x + delta_x1 * r1[0].evalf()
                y = y + delta_x2 * r1[0].evalf()
            else:
                print("\n Note that no root found for h(r) = ", expr.subs({x1: delta_x1 * r + x, x2: delta_x2 * r + y}), end=',')
                print(" so iteration is over right here\n")
                break

        print("--- Result ------------------------------------------")
        print(" The maximization point (x, y) is ({0}, {1})".format(x,  y))
        print(" Maximization of f(x1, x2) = ", expr.subs({x1: x, x2: y}))

        return True
