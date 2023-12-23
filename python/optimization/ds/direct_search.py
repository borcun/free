"""
  The idea of direct search method is to identify the 'interval of uncertainty' known
to include the optimum solution point. The procedure locates the optimum by iteratively
narrowing the interval of uncertainty to a desired level of accuracy.
  Direct search methods apply primarily to strictly unimodal single-variable functions.
Two closely related search algorithms are presented: dichotomous and golden section. These
two methods start with the initial interval of uncertainty I0 = (a, b)
"""

import sys

class DirectSearch:
    def __init__(self):
        # equation string that is set from file content
        self.equation = ""
        # first and second points supplied by user
        self.point1 = []
        self.point2 = []
        # epsilon value (aka delta in dichotomous approach) supplied by user
        self.epsilon = 0

    """
    Function that gets equation for Direct Search Method algorithm
    Params:
      path - input file path
    Return:
      True if the file is read, otherwise False
    """
    def getEquation(self, path):
        try:
            pFile = open(path, "r")
            self.equation = pFile.readline()
            pFile.close()
            
            return True
        except:
            print("Could not read", path)
        
        return False

    """
    Function that gets input parameters such as X, Y coordinate and iteration count
    """
    def getInputParameters(self):
        while True:
            try:
                print(" Enter x of first point : ", end = '')
                self.point1.append(float(input()))
                print(" Enter y of first point : ", end = '')
                self.point1.append(float(input()))
                print(" Enter x of second point : ", end = '')
                self.point2.append(float(input()))
                print(" Enter y of second point : ", end = '')
                self.point2.append(float(input()))
                print(" Enter epsilon value  : ", end = '')
                self.epsilon = float(input())

                if self.epsilon < 1:
                    print("\n Please enter positive number for epsilon\n")
                else:
                    break
                
            except:
                print("\n Could not convert entries to float numbers\n")
                pass    
