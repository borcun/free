# this file contains shift_reverse function that shifts all word
# of sentence from user, then reverses all word and create a new string

from sys import stdin

# brief  : this function that shifts words string array to right 
#          and reverses each word of words
# param  : words - string array
# return : -
def shift_reverse( words ) :
    # size is length of words - 1 because last word is jumped when processing
    size = len( words ) - 1
    # shifted and reversed string array
    temp = []

    # reverse and add last word into empty temp array
    temp.append( words[ len( words ) - 1 ][ ::-1 ] )

    for i in range( 0, len( words ) - 1 ) :
        temp.append( words[ i ][ ::-1 ] )

    for word in temp:
        print( "%s " % word, end="" )
    print()

# end of shift_reverse function

sentence = stdin.readline()
words = sentence.split() # by default, delimeter is space for split function

shift_reverse( words )
