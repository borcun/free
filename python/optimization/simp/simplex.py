import sys
from . import utility

class Simplex:
    def __init__(self):
        self.prob_type = 'x'
        self.coefficients = []
        self.util = utility.SimplexUtility()

    """
    Function that gets input for Simplex Method algorithm
    Params:
      path - input file path
    Return:
      True if the file is read, otherwise False
    """
    def getInput(self, path):
        self.coefficients = self.util.parse(path)
        return None != self.coefficients

    """
    Function that executes Simplex Method algorithm
    """
    def execute(self, path):
        if not self.getInput(path):
            return False
        
        print("\n * simplex method executed to get solution")
        print(self.coefficients)
            
        return True
