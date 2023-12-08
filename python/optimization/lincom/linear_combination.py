import sys

def execute(ptype, pcoef):
    if None == ptype or None == pcoef:
        print("Could not execute linear combination method due to invalid parameters")
        return False
    else:
        if 'x' == ptype:
            print("\n * linear combination method executed to get maximum solution")
        elif 'n' == ptype:
            print("\n * linear combination method executed to get minimum solution")
        else:
            print("invalid problem type")

        print(coefficients)

        return True
