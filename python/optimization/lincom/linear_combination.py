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

class LinearCombinations:
    def __init__(self):
        pass
    
    def execute(self, ptype, pcoef):
        if None == ptype or None == pcoef:
            print("Could not execute linear combination method due to invalid parameters")
            return False
        else:
            if 'x' == ptype:
                print("\n * linear combination method executed to get maximum solution")
            elif 'n' == ptype:
                print("\n * linear combination method executed to get minimum solution")
            else:
                print("invalid problem type")

        print(coefficients)
        return True
