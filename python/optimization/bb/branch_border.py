import sys

class BranchBound:
    def __init__(self):
        pass
    
    def execute(self, ptype, pcoef):
        if None == ptype or None == pcoef:
            print("Could not execute branch and bound method due to invalid parameters")
            return False
        else:
            if 'x' == ptype:
                print("\n * branch and bound method executed to get maximum solution")
            elif 'n' == ptype:
                print("\n * branch and bound method executed to get minimum solution")
            else:
                print("invalid problem type")

        print(coefficients)
        return True
