#!/usr/bin/python3

import sys
import utility as util

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
        self.algorithms = [simp.Simplex(), bb.BranchBound(), nr.NewtonRaphson(), disc.Discrete(), ds.DirectSearch(), lincom.LinearCombinations()]
        self.algUtil = util.AlgUtil()
        self.ptype = None
        self.pcoef = None
    
    """
    The function that gets problem parameters such as coefficients, types

    Return:
    problem type and coefficients
    """
    def getInputParameters(self):
        self.ptype = None
        self.pcoef = None
    
        while True:
            print(" Enter 'q', then press return to terminate algorithm selection")
            print(" Enter problem type, Ma[x]imization or Mi[n]imization : ", end = '')
            
            self.ptype = input()
            
            # exit from problem type
            if 'q' == self.ptype.lower():
                break
            elif 'x' != self.ptype.lower() and 'n' != self.ptype.lower():
                print("\n Error: invalid problem type, please enter again\n")
                continue
            
            print(" Enter input file path: ", end = '')
            path = input()
            
            # exit from path
            if 'q' == path.lower():
                break
            
            self.pcoef = self.algUtil.parse(path)
            
            if None == self.pcoef:
                print("\n Error: invalid file path, please enter again\n")
            else:
                break

            
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
                    self.getInputParameters()

                    if 1 == opt:
                        print("\nSIMPLEX ALGORITHM")
                        self.algUtil.print()
                    elif 2 == opt:
                        print("\nBRANCH and BOUND ALGORITHM")
                        self.algUtil.print()
                    elif 3 == opt:
                        print("\nNEWTON-RAPHSON ALGORITHM")
                        self.algUtil.print()
                    elif 4 == opt:
                        print("\nDIRECT SEARCH ALGORITHM")
                    elif 5 == opt:
                        print("\nDISCRETE ALGORITHM")
                    elif 6 == opt:
                        print("\nLINEAR COMBINATION ALGORITHM")

                    if None != self.ptype and None != self.pcoef:
                        self.algorithms[opt - 1].execute(self.ptype, self.pcoef)
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
