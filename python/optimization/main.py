#!/usr/bin/python3

from simp import simplex as simp
from bb import branch_bound as bb
from nr import newton_raphson as nr
from sep import separable as sep
from ds import direct_search as ds
from lincom import linear_combination as lincom
from grad import gradient as grad

"""
main function
"""
def main():
    simplex = simp.Simplex()
    branch_bound = bb.BranchBound()
    newton_raphson = nr.NewtonRaphson()
    separable = sep.Separable()
    direct_search = ds.DirectSearch()
    linear_combination = lincom.LinearCombinations()
    gradient = grad.Gradient()
    show_menu_flag = True

    while show_menu_flag:
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
        print("\n Please enter an option, then press return key : ", end='')

        try:
            opt = int(input())

            if opt < 0 or opt > 8:
                print("\n Error: invalid input, please enter again")
            elif 8 == opt:
                show_menu_flag = False
            else:
                if 1 == opt:
                    print("\nSIMPLEX ALGORITHM")
                    print(" Enter file path for Simplex algorithm: ", end='')

                    if simplex.read(input()):
                        simplex.solve()

                elif 2 == opt:
                    print("\nBRANCH BOUND ALGORITHM")
                    print(" Enter file path for Branch Bound algorithm: ", end='')

                    if branch_bound.read(input()):
                        branch_bound.solve()

                elif 3 == opt:
                    print("\nNEWTON-RAPHSON ALGORITHM")
                    print(" Enter file path for Newton-Raphson algorithm: ", end='')

                    if newton_raphson.read(input()):
                        newton_raphson.solve()

                elif 4 == opt:
                    print("\nDIRECT SEARCH ALGORITHM")
                    print(" Enter file path for Direct Search algorithm: ", end='')

                    if direct_search.read(input()):
                        direct_search.interact()
                        direct_search.solve()

                elif 5 == opt:
                    print("\nSEPARABLE ALGORITHM")
                    print(" Enter file path for Separable programming algorithm: ", end='')

                    if separable.read(input()):
                        separable.solve()

                elif 6 == opt:
                    print("\nLINEAR COMBINATION ALGORITHM")
                    print(" Enter file path for Linear Combinations algorithm: ", end='')

                    if linear_combination.read(input()):
                        linear_combination.interact()
                        linear_combination.solve()

                elif 7 == opt:
                    print("\nGRADIENT ALGORITHM")
                    print(" Enter file path for gradient algorithm: ", end='')

                    if gradient.read(input()):
                        gradient.interact()
                        gradient.solve()
        except:
            pass


if __name__ == "__main__":
    main()
