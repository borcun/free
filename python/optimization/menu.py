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
The function that gets problem parameters such as coefficients, types

Return:
 problem type and coefficients
"""
def getInputParameters():
    ptype = None
    pcoef = None
    
    while True:
        print(" Enter 'q', then press return to terminate algorithm selection")
        print(" Enter problem type, Ma[x]imization or Mi[n]imization : ", end = '')

        ptype = input()

        # exit from problem type
        if 'q' == ptype.lower():
            break
        elif 'x' != ptype.lower() and 'n' != ptype.lower():
            print("\n Error: invalid problem type, please enter again\n")
            continue
            
        print(" Enter input file path: ", end = '')
        path = input()

        # exit from path
        if 'q' == path.lower():
            break

        pcoef = util.parse(path)

        if None == pcoef:
            print("\n Error: invalid file path, please enter again\n")
        else:
            break

    return ptype, pcoef
    
"""
The function that shows menu
"""
def showMenu():
    showMenuFlag = True
    
    while showMenuFlag:
        print("\nALGORITHMS")
        print(" [0] Show Menu")
        print(" [1] Simplex Algorithm")
        print(" [2] Branch/Border Algorithm")
        print(" [3] Newton-Raphson Method")
        print(" [4] Direct Search Method")
        print(" [5] Discrete Programming")
        print(" [6] Linear Combinations Method")
        print(" [7] Terminate Application")
        print("\n Please enter an option, then press return key : ", end = '')

        try:
            opt = int(input())
        
            if 0 != opt:
                if 1 == opt:
                    print("\nSIMPLEX ALGORITHM")
                    ptype, pcoef = getInputParameters()

                    if None != ptype and None != pcoef:
                        simp.execute(ptype, pcoef)
                elif 2 == opt:
                    print("\nBRANCH and BOUND ALGORITHM")
                    ptype, pcoef = getInputParameters()

                    if None != ptype and None != pcoef:
                        bb.execute(ptype, pcoef)
                elif 3 == opt:
                    print("\nNEWTON-RAPHSON ALGORITHM")
                    ptype, pcoef = getInputParameters()

                    if None != ptype and None != pcoef:
                        nr.execute(ptype, pcoef)
                elif 4 == opt:
                    print("\nDIRECT SEARCH ALGORITHM")
                    ptype, pcoef = getInputParameters()

                    if None != ptype and None != pcoef:
                        ds.execute(ptype, pcoef)
                elif 5 == opt:
                    print("\nDISCRETE ALGORITHM")
                    ptype, pcoef = getInputParameters()

                    if None != ptype and None != pcoef:
                        disc.execute(ptype, pcoef)
                elif 6 == opt:
                    print("\nLINEAR COMBINATION ALGORITHM")
                    ptype, pcoef = getInputParameters()

                    if None != ptype and None != pcoef:
                        lincom.execute(ptype, pcoef)
                elif 7 == opt:
                    showMenuFlag = False
                else:
                    print("\n Error: invalid input, please enter again")
        except:
            pass

"""
main function
"""
def main():    
    showMenu()


if __name__ == "__main__":
    main()
