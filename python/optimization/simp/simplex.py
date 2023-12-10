import sys
import utility

class Simplex:
    def __init__(self):
        self.prob_type = 'x'
        self.coefficients = []
        self.isInputAvailable = False
        self.util = utility.AlgUtil()

    """
    Function that gets input for Simplex Method algorithm
    Params:
      path - input file path
    Return:
      True if the file is read, otherwise False
    """
    def getInput(self, path):
        self.coefficients = self.util.parse(path)

        if None == self.coefficients:
            self.isInputAvailable = False
        else:
            self.isInputAvailable = True

        return self.isInputAvailable

    """
    Function that executes Simplex Method algorithm
    """
    def execute(self):
        if self.isInputAvailable:
            print("\n * simplex method executed to get solution")
            print(self.coefficients)
            
        return True
