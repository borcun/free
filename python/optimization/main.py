#!/usr/bin/python3

import sys
from simp import simplex as simp
from bb import branch_bound as bb
from nr import newton_raphson as nr
from sep import separable as sep
from ds import direct_search as ds
from lincom import linear_combination as lincom
from grad import gradient as grad

"""
Algorithms class
"""
class Algorithms:
    """
    Default constructor
    """
    def __init__(self):
        self.simp = simp.Simplex()
        self.bb = bb.BranchBound()
        self.nr = nr.NewtonRaphson()
        self.sep = sep.Separable()
        self.ds = ds.DirectSearch()
        self.lincom = lincom.LinearCombinations()
        self.grad = grad.Gradient()

        
    """
    The function that shows menu
    """
    def showMenu(self):
        showMenuFlag = True
            
        while showMenuFlag:
            print("\nALGORITHMS")
            print(" [0] Show Menu")
            print(" [1] Simplex Algorithm")
            print(" [2] Branch Bound Algorithm")
            print(" [3] Newton-Raphson Method")
            print(" [4] Direct Search Method")
            print(" [5] Separable Programming")
            print(" [6] Linear Combinations Method")
            print(" [7] Gradient Method")
            print(" [8] Terminate Application")
            print("\n Please enter an option, then press return key : ", end = '')

            try:
                opt = int(input())

                if opt < 0 or opt > 8:
                    print("\n Error: invalid input, please enter again")
                elif 8 == opt:
                    showMenuFlag = False
                else:
                    if 1 == opt:
                        print("\nSIMPLEX ALGORITHM")
                        print(" Enter file path for Simplex algorithm: ", end = '')                        
                        self.simp.execute(input())
                    elif 2 == opt:
                        print("\nBRANCH BOUND ALGORITHM")
                        print(" Enter file path for Branch Bound algorithm: ", end = '')                        
                        self.bb.execute(input())
                    elif 3 == opt:
                        print("\nNEWTON-RAPHSON ALGORITHM")
                        print(" Enter file path for Newton-Raphson algorithm: ", end = '')
                        self.nr.execute(input())
                    elif 4 == opt:
                        print("\nDIRECT SEARCH ALGORITHM")
                        print(" Enter file path for Direct Search algorithm: ", end = '')
                        self.ds.execute(input())
                    elif 5 == opt:
                        print("\nSEPARABLE ALGORITHM")
                        print(" Enter file path for Separable programming algorithm: ", end = '')
                        self.sep.execute(input())                        
                    elif 6 == opt:
                        print("\nLINEAR COMBINATION ALGORITHM")
                        print(" Enter file path for Linear Combinations algorithm: ", end = '')
                        self.lincom.execute(input())
                    elif 7 == opt:
                        print("\nGRADIENT ALGORITHM")
                        print(" Enter file path for gradient algorithm: ", end = '')
                        self.grad.execute(input())
            except:
                pass

            
"""
main function
"""
def main():
    alg = Algorithms()
    alg.showMenu()


if __name__ == "__main__":
    main()
