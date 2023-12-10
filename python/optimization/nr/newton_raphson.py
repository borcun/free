import sys
from . import utility

class NewtonRaphson:
    def __init__(self):
        self.prob_type = 'x'
        self.coefficients = []
        self.util = utility.NewtonRaphsonUtility()

    """
    Function that gets input for Newton Raphson Method algorithm
    Params:
      path - input file path
    Return:
      True if the file is read, otherwise False
    """
    def getInput(self, path):
        self.coefficients = self.util.parse(path)
        return None != self.coefficients

    """
    Function that executes Newton Raphson Method algorithm
    """
    def execute(self, path):
        if not self.getInput(path):
            return False
        
        print("\n * newton-raphson method executed to get solution")
        self.util.print()
            
        return True
