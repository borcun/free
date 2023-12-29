"""
The base class for all algorithms
"""
class Algorithm(object):
    """
    Default constructor
    """
    def __init__(self):
        pass

    """
    The function that reads input file for getting algorithm parameters
    
    Params:
      path - input file path
    """
    def read(self, path):
        return False

    """
    """
    def interact(self):
        pass

    """
    The function that executes solution for the algorithm
    """
    def solve(self):
        return False
