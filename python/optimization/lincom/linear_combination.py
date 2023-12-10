"""
  The steepest-ascent method is a method for unconstrained mathematical optimization. It
is used for optimizing twice continuously differentiable function. The idea is to generate
successive points in the direction of the gradient of the function. Termination of the
gradient method occurs at the point where the gradient vector becomes null. This is only
a necessary condition for optimality.

  The linear combinations method is based on the steepest-ascent (aka gradient) method.
However, the direction specified by the gradient vector may not yield a feasible solution
for the constrained problem. Also, the gradient vector will not necessarily be null at
the optimum (constrained) point. The steepest-ascent method thus must be modified to
handle the constrained case.
"""

import sys
from sympy import *

class LinearCombinations:
    def __init__(self):
        pass
    
    def execute(self, equation):
        x1, x2 = symbols('x1 x2')
        expr = equation

        print(diff(expr, x1))
        print(diff(expr, x1).subs([(x1, 2), (x2, 3)]))
        
        return True
