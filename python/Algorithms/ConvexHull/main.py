#!/usr/bin/python3

import os
import stat
from convexHull import ConvexHull

READ_FROM_FILE = 1
SOLVE_CONVEX_HULL = 2
EXIT_FROM_APP = 3

# menu function
def menu():
    return int( input( "[1] Read points from file\n"
                       "[2] Solve Convex Hull\n"
                       "[3] Exit\n"
                       " -> " ) )

def entryPoint():
    convex_hull = ConvexHull()
    
    while True:
        choice = menu()
        
        if choice == READ_FROM_FILE:
            file_path = str( input( "enter file path: " ) )

            if 0 == len( file_path ):
                print( "Enter a valid file path!\n" )
            elif os.access( file_path, os.F_OK ):
                convex_hull.readFile( file_path )
            else:
                print( "File is not accessible!\n" )                
        elif choice == SOLVE_CONVEX_HULL:
            convex_hull.solve()
        elif choice == EXIT_FROM_APP:
            return
        else:
            print( " Invalid Choice ! Please enter again\n" )


entryPoint()
        
