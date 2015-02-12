import os;
import sys;

def enumetor():
    if os.name == "nt":
        print os.name
        print "windows-based os"
    else:
        print os.name

obj = enumetor()
file = open("test.txt", "w")
file.write(str(obj))
