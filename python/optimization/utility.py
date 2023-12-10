""" the module is used to parse csv file that include input parameters for the project """

import math
import pandas as pd

class AlgUtil:
    coefficients = []

    def __init__(self):
        pass
    
    """
    The function that parses csv file, then get input parameter inside
    
    Params:
    path - csv file path
    Return:
    coefficients if succeed, otherwise None
    """
    def parseLinearParameters(self, path):
        try:
            df = pd.read_csv(path)
            self.coefficients.clear()
            
            for row in range(len(df)):
                self.coefficients.append(df.iloc[row].to_list())
                
            return self.coefficients
        except:
            return None

    """
    """
    def parseNonLinearParameters(self, path):
        return '4*x1 + 6*x2 - 2*x1**2 - 2*x1*x2 - 2*x2**2'

        
    """
    Function that prints coefficients if problem is linear
    Remark:
      It is not suitable to print non-linear algorithm coefficients
    """
    def print(self):
        if 0 > len(self.coefficients):
            print("Coefficient list is empty, nothing to parse")
        else:            
            for i in range(len(self.coefficients)):
                # remove nan values from the list
                func_coef = [c for c in self.coefficients[i] if not math.isnan(c)]

                if 0 == i:                    
                    print("\n Objective Function    : ", end = '')                    

                    for j in range(len(func_coef)):
                        if not math.isnan(func_coef[j]):
                            print("{0}x{1}".format(func_coef[j], j + 1), end = '')
                        
                            if j < len(func_coef) - 1:
                                print(" + ", end = '')
                else:
                    print(" Constraint Function {0} : ".format(i), end = '')

                    for j in range(len(func_coef)):
                        if not math.isnan(func_coef[j]):                        
                            if j == len(func_coef) - 1:
                                print(" <>= {}".format(func_coef[j]), end = '')
                            else:    
                                print("{0}x{1}".format(func_coef[j], j + 1), end = '')
                            
                                if j < len(func_coef) - 2:
                                    print(" + ", end = '')

                print('')
