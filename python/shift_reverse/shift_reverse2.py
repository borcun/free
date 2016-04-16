# reverse function reverses string parameter
def reverse(str) :
    # print string reversely
    for i in range(len(str)-1, -1, -1):
        print("%c" % str[i], end="")
    # space between words
    print(" ", end="")

# right_shift function shifts all words in a line to right way
def right_shift(line) :
    line_arr = line.split(" ")

    # add last element head of array
    line_arr.insert(0, line_arr[len(line_arr) - 1])
    # remove last element, complete right shifting
    line_arr.pop();

    return line_arr

# shift_reverse function is a wrapper function for shift and reverse functions.
def shift_reverse(line) :
    words = right_shift(line)

    for i in range(len(words)) : 
        reverse(words[i])
    print("")


shift_reverse( input("enter the sentence : ") )
