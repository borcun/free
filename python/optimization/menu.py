#!/usr/bin/python3

import sys

def simplex():
    print("\n * simplex method executed\n")

def branch_and_border():
    print("\n *branch/border algorithm executed\n")

def newton_raphson():
    print("\n *newton raphson method executed\n")

def direct_search():
    print("\n *direct search method executed\n")

def discrete():
    print("\n *discrete programming executed\n")
    
def linear_combination():
    print("\n *linear combinations method executed\n")

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
            simplex()
        elif 2 == opt:
            branch_and_border()
        elif 3 == opt:
            newton_raphson()
        elif 4 == opt:
            direct_search()
        elif 5 == opt:
            discrete()
        elif 6 == opt:
            linear_combination()
        elif 7 == opt:
            exit()
        else:
            print("\n Warning: invalid input!\n")
        
def main():    
    menu()

if __name__ == "__main__":
    main()
