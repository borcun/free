from sympy import *
import csv

init_printing(use_unicode=True)

class NewtonRaphson:
    def __init__(self):
        self.data = []

    """
    """
    def getEquations(self, path):
        try:
            with open(path, 'r', newline='') as file:
                csv_data = csv.reader(file, delimiter=';')
                self.data.clear()

                for line in csv_data:
                    self.data = line

                return True
        except FileNotFoundError:
            print("Could not found", path)
            return False
        except Exception as err:
            print("An unexpected error occurred:", err)
            return False

    """
    """
    def execute(self, path):
        if not self.getEquations(path):
            return False

        func = self.data[0]
        x0 = float(self.data[1])
        e = float(self.data[2])
        x = symbols('x')
        f = Lambda(x,eval(func))
        g = Lambda(x, diff(f(x), x))
        # g is the derivative function
        step = 1
        flag = 1
        condition = True
        x1 = 0

        while condition:
            if g(x0) == 0.0:
                print('Division by zero error!')
                break

            x1 = x0 - f(x0)/g(x0)
            print('Iteration-%d, x1 = %0.6f, f(x1) = %0.6f and g(x1) = %0.6f' % (step, x1, f(x1), g(x1)))
            x0 = x1
            step = step + 1

            if step > 10:
                flag = 0
                break

            condition = abs(f(x1)) > e

        if 1 == flag:
            print('\nRequired root is: %0.8f' % x1)
        else:
            print('\nNot Convergent.')
