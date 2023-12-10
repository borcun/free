#!/usr/bin/python3

import sys
from simp import simplex as simp
from bb import branch_border as bb
from nr import newton_raphson as nr
from disc import discrete as disc
from ds import direct_search as ds
from lincom import linear_combination as lincom


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
        self.disc = disc.Discrete()
        self.ds = ds.DirectSearch()
        self.lincom = lincom.LinearCombinations()

        
    """
    The function that shows menu
    """
    def showMenu(self):
        showMenuFlag = True
            
        while showMenuFlag:
            print("\nALGORITHMS")
            print(" [0] Show Menu")
            print(" [1] Simplex Algorithm")
            print(" [2] Branch/Border Algorithm")
            print(" [3] Newton-Raphson Method")
            print(" [4] Discrete Programming")
            print(" [5] Direct Search Method")
            print(" [6] Linear Combinations Method")
            print(" [7] Terminate Application")
            print("\n Please enter an option, then press return key : ", end = '')

            try:
                opt = int(input())

                if opt < 0 or opt > 7:
                    print("\n Error: invalid input, please enter again")
                elif 7 == opt:
                    showMenuFlag = False
                else:
                    if 1 == opt:
                        print("\nSIMPLEX ALGORITHM")
                        print(" Enter file path for simplex algorithm: ", end = '')                        
                        self.simp.execute(input())
                    elif 2 == opt:
                        print("\nBRANCH and BOUND ALGORITHM")
                    elif 3 == opt:
                        print("\nNEWTON-RAPHSON ALGORITHM")
                    elif 4 == opt:
                        print("\nDIRECT SEARCH ALGORITHM")
                    elif 5 == opt:
                        print("\nDISCRETE ALGORITHM")
                    elif 6 == opt:
                        print("\nLINEAR COMBINATION ALGORITHM")
                        print(" Enter file path for linear combinations algorithm: ", end = '')
                        self.lincom.execute(input())
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
