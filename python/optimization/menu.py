#!/usr/bin/python3

import simplex as simp
import branch_border as bb
import newton_raphson as nr
import discrete as dsc
import direct_search as dsrc
import linear_combination as lincom

def menu():
    while True:
        print("ALGORITHMS")
        print(" [1] Simplex Algorithm")
        print(" [2] Branch/Border Algorithm")
        print(" [3] Newton-Raphson Method")
        print(" [4] Direct Search Method")
        print(" [5] Discrete Programming")
        print(" [6] Linear Combinations Method")
        print(" [7] Exit")
        print("\n Please enter an algorithm number")
        print("  > ", end = '')
        
        opt = int(input())

        if 1 == opt:
            simp.execute()
        elif 2 == opt:
            bb.execute()
        elif 3 == opt:
            nr.execute()
        elif 4 == opt:
            dsrc.execute()
        elif 5 == opt:
            dsc.execute()
        elif 6 == opt:
            lincom.execute()
        elif 7 == opt:
            exit()
        else:
            print("\n Warning: invalid input!\n")
        
def main():    
    menu()

if __name__ == "__main__":
    main()
