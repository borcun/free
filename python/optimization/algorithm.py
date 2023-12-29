"""
The base class for all optimization algorithms
"""
class Algorithm(object):
    """
    Default constructor
    """
    def __init__(self):
        pass

    """
    The function that reads input file for getting algorithm equations/coefficients
    
    Params:
      path - input file path
    Return:
      True if the input file is read successfully, False otherwise
    """
    def read(self, path):
        return False

    """
    The function that interact with user on console to get immutable input parameters of the algorithm
    """
    def interact(self):
        pass

    """
    The function that executes solution method for the algorithm
    
    Return:
      True if the solution is executed successfully, False otherwise
    """
    def solve(self):
        return False
