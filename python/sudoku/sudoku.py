# this program finds possible number values in sudoku where two indices are indicated.

MAX_ROW = 9
MAX_COL = 9
SEPARATOR_COUNT = 21
SEPARATOR_INDICES = ( 2, 5 )
NUMBERS = range( 1, 10 )
SYMBOL = '.'

# brief  : function that prints sudoku board on screen
# param  : board - sudoku board
# return : - 
def print_board( board ) :
    for i in range( MAX_ROW ) : 
        for j in range( MAX_COL ) : 
            print( "%c " % board[i][j], end="" )
            
            if j in SEPARATOR_INDICES :
                print( "%s" % "| ", end="" )
        if i in SEPARATOR_INDICES : 
            print("")
            for j in range( SEPARATOR_COUNT ) :
                print( "%c" % '-', end="" )
        print("")
# end of print_board function

# brief  : function that gets possible numbers which the indices are indicated on board
# param  : board - sudoku board
# param  : row - row index
# param  : col - column index
# return : possible numbers or -1 if not possible 
def get_possible_numbers( board, row, col ) :
    # array which will store impossible numbers to eliminate from possibles
    impossible_numbers = []

    if row < 0 or col < 0 or row >= MAX_ROW or col >= MAX_COL :
        print( "error : invalid index" )
        return

    if SYMBOL != board[ row ][ col ] :
        print( "the board(%d, %d) always stores number in these indices" % ( row, col ) )
        return

    for i in range( MAX_ROW ) :
        for j in range( MAX_COL ) :
            # if correct row and column is found and the symbol is not dot
            # that value is one from impossibles
            if ( i == row or j == col ) and SYMBOL != board[i][j] :
                # ord function converts character to integer such as from '2' to 50 
                # -48 is for find integer value of ascii value. 
                # thus, impossible numbers are made as integer to eliminate easy.
                impossible_numbers.append( ( ord( board[i][j] ) - 48 ) )

    # scan all number list and find possible numbers not inside impossible numbers array
    for i in NUMBERS : 
        if not ( i in impossible_numbers ) :
            print( i )
 
# end of get_possible_numbers function

board = [ [SYMBOL for i in range(9) ] for i in range(9) ]

board[0][0] = '4'
board[0][2] = '7'
board[0][5] = '8'
board[0][6] = '9'
board[0][7] = '6'
board[0][8] = '2'
board[1][2] = '1'
board[1][4] = '2'
board[2][0] = '5'
board[2][3] = '7'
board[3][0] = '7'
board[3][1] = '5'
board[3][8] = '4'
board[4][4] = '4'
board[5][0] = '6'
board[5][7] = '9'
board[5][8] = '8'
board[6][5] = '1'
board[6][8] = '6'
board[7][4] = '5'
board[7][6] = '8'
board[8][0] = '1'
board[8][1] = '2'
board[8][2] = '9'
board[8][3] = '4'
board[8][6] = '3'
board[8][8] = '5'

print_board( board )

get_possible_numbers( board, 3, 4 )
