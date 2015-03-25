package ccs;

import java.util.Scanner;

// CCSGame class
public class CCSGame {
    // CCSRule class
    private static class CCSRule {
	public static int MAX_MOVE = 0;
	public static int SCORE = 0;
	public static final int MAX_SHUFFLE = 3;
	public static final int MIN_SEQUENCE = 3;
        public static final int PREPARE = 10;
    }

    private CCSTable m_table;
    private int m_score;
    private int m_move;
    // game status
    private enum status_t {
	WIN, LOST, CONTINUE
    };
    // before game, play time
    private enum game_t {
        BEFORE, PLAYING
    }

    // default constructor
    public CCSGame() {
        System.out.println( "Welcome to Candy Crush Saga\n" );
    }

    // method that starts game
    public void start( int row, int col, char[] icons, int move, int score ) {
        System.out.println( "\nLoading Game" );
	m_table = new CCSTable( row, col );
	m_table.create( icons );
        
	CCSRule.MAX_MOVE = move;
        CCSRule.SCORE = score;
        m_score = 0;
	m_move = 0;
    }

    public void play() {
	Scanner scanner = new Scanner( System.in );
	boolean is_continue = true;
        boolean is_valid = true;
        String[] coordinate;
        
        prepare();
        
	while( is_continue ) {
	    switch( check() ) {
	    case WIN:
		System.out.println( "Congrulation. The CCS game finished." );
		System.out.printf( "You moved %d times, score is %d, \n", m_move ,m_score );
		is_continue = false;
		break;
	    case LOST:
		System.out.println( "Failure. The CCS game finished." );
		System.out.printf( "You moved %d times, score is %d, \n", m_move ,m_score );
		is_continue = false;
		break;
	    case CONTINUE:
                m_table.print();

                do {
                    System.out.printf( "Move : %d (%d), Score : %d (%d)\n", m_move, CCSRule.MAX_MOVE, m_score, CCSRule.SCORE );
                    System.out.print("Enter a move : ");
                    String move = scanner.next();
                    
                    System.out.println();
                    coordinate = move.split(",");
                    
                    if( coordinate.length == 4 ) {
                        int counter = 0;
                        for(counter=0 ; counter < coordinate.length ; ++counter) {
                            if( Character.isDigit( Integer.parseInt(coordinate[counter]) ) ) {
                                System.err.println("invalid move format");
                                break;
                            }
                            else if( counter % 2 == 0 ) {
                                if( Integer.parseInt(coordinate[counter]) > m_table.row() ) {
                                    System.err.println("invalid row value : " + coordinate[counter]);
                                    break;
                                }
                                else if( Integer.parseInt(coordinate[counter]) > m_table.column()) {
                                    System.err.println("invalid column value : " + coordinate[counter]);
                                    break;
                                }
                            }
                        }
                        
                        if( !coordinate[0].equals(coordinate[2]) || !coordinate[1].equals(coordinate[3]) )
                            System.err.println("invalid move format");
                        else if( coordinate[0].equals(coordinate[2]) && coordinate[1].equals(coordinate[3]) )
                            System.err.println("invalid move format");                            
                        else if( coordinate[0].equals(coordinate[2]) ) {
                            int coor1 = Integer.parseInt(coordinate[1]);
                            int coor2 = Integer.parseInt(coordinate[3]);
                            
                            if( (coor1 != coor2 + 1) || (coor1 != coor2 - 1) )
                                System.err.println("invalid move format");                            
                        }
                        else if( coordinate[1].equals(coordinate[3]) ) {
                            int coor1 = Integer.parseInt(coordinate[0]);
                            int coor2 = Integer.parseInt(coordinate[2]);
                            
                            if( (coor1 != coor2 + 1) || (coor1 != coor2 - 1) )
                                System.err.println("invalid move format");                            
                        }
                        else if( 4 == counter )
                            is_valid = false;
                    } else
                        System.err.println("missing parameter");
                }
                while( is_valid );

                is_valid = true;
		// if table isn't steady, shuffle it until shuffle reaches maximum shuffle
		update( Integer.parseInt( coordinate[0] ), 
			Integer.parseInt( coordinate[1] ), 
			Integer.parseInt( coordinate[2] ), 
			Integer.parseInt( coordinate[3] ) );

                ++m_move;
		break;
	    } // end of switch
	} // end of while
    }
    
    // method that prepares game table
    private void prepare() {
        char[][] table = m_table.getTable();
        boolean is_row_operated = false;
        boolean is_col_operated = false;
        int try_prepare = 0;
        boolean extra_row = false;
        boolean extra_col = false;

	do {            
            while( try_prepare++ < CCSRule.PREPARE ) {              
                is_row_operated = false;

                // find operation starts from last line, because
                // any operation which performs in last line effects top lines
                for( int i = table.length - 1 ; i >= 0 ; --i ) {
                    int[] section = new int[2];

                    while( find( m_table.getRow( i ), section ) ) {
                        m_table.print();
                        shift( i, section, CCSTable.line_t.ROW );
                        m_table.print();
                        is_row_operated = true;
                        extra_row = true;
                        i = table.length - 1;
                    }
                    
                    if( extra_row ) {
                        table = m_table.getTable();
                    }
                    extra_row = false;
                }

                is_col_operated = false;
                m_table.print();

                for( int j = 0 ; j < table[0].length ; ++j ) {
                    int[] section = new int[2];

                    while( find( m_table.getColumn( j ), section ) ) {
                        shift( j, section, CCSTable.line_t.COLUMN );
                        is_col_operated = true;
                        extra_col = true;
                        j = 0;
                    }
                    
                    if( extra_col ) {
                        table = m_table.getTable();
                    }
                    extra_col = false;
                }

                m_table.print();

                if( !is_row_operated && !is_col_operated )
                    ++try_prepare;
                else
                    try_prepare = 0;
            }
	} while( is_row_operated || is_col_operated );        
        
        System.out.println( try_prepare );
        System.out.println( is_row_operated );
        System.out.println( is_col_operated );
    }

    // method that checks whether the game continues
    private status_t check() {
	if( CCSRule.MAX_MOVE > m_move && m_score >= CCSRule.SCORE )
	    return status_t.WIN;
	else if( CCSRule.MAX_MOVE == m_move && m_score < CCSRule.SCORE )
	    return status_t.LOST;
	else
	    return status_t.CONTINUE;
    }

    // method that updates table
    private void update( int _row, int _col, int row, int col ) {
	char[][] table = m_table.getTable();
	int shuffle = 0;
	boolean is_ok = false;
        boolean is_row_operated = false;
        boolean is_col_operated = false;
        boolean can_continue = true;

	do {
            // if the shuffle operation is performed, don't swap again
            if( shuffle == 0 ) {
                char temp = table[ _row ][ _col ];
                table[ _row ][ _col ] = table[ row ][ col ];
                table[ row ][ col ] = temp;

                m_table.setTable( table );
            }
            
            while( can_continue ) {
                m_table.print();

                is_row_operated = false;

                // find operation starts from last line, because
                // any operation which performs in last line effects top lines
                for( int i = table.length - 1 ; i >= 0 ; --i ) {
                    int[] section = new int[2];

                    while( find( m_table.getRow( i ), section ) && !is_row_operated ) {
                        String str = String.copyValueOf(m_table.getRow( i ));
                        System.out.printf( "%d. row, from %d to %d : %s\n\n", i, section[0], section[0]+section[1], 
                                str.substring( section[0], section[0]+section[1] ) );
                        shift( i, section, CCSTable.line_t.ROW );
                        m_table.print();
                        is_row_operated = true;
                        m_score += Math.pow(section[1] - section[0], 2);
                        // start again
                        i = table.length - 1;
                    }
                }

                is_col_operated = false;

                for( int i = 0 ; i < table[0].length ; ++i ) {
                    int[] section = new int[2];

                    while( find( m_table.getColumn( i ), section ) && !is_col_operated ) {
                        String str = String.copyValueOf( m_table.getColumn( i ) );
                        System.out.printf( "%d. column, from %d to %d : %s\n\n", i, section[0], section[0]+section[1], 
                                str.substring(section[0], section[0]+section[1]) );
                        shift( i, section, CCSTable.line_t.COLUMN );
                        m_table.print();
                        m_score += Math.pow(section[1] - section[0], 2);
                        is_col_operated = true;
                    }
                }

                if( !(is_row_operated || is_col_operated) ) {
                    if( ++shuffle != CCSRule.MAX_SHUFFLE ) {
                        m_table.shuffle();
                        can_continue = true;
                    }
                    else
                        can_continue = false;
                }
                else
                    can_continue = is_row_operated || is_col_operated;
            }
	} while( shuffle != CCSRule.MAX_SHUFFLE && !(is_row_operated || is_col_operated) );
    }

    // method that finds sequence
    private boolean find( char[] line, int[] section ) {
        String str = String.valueOf(line);           

        for( int j = 1 ; j <= str.length() - CCSRule.MIN_SEQUENCE + 1 ; ++j ) {
            char ch = str.charAt(j-1);
            String kernel = "";

            for( int k = 0 ; k <= str.length() - j ; ++k)
                kernel += ch;

            if( -1 != ( section[0] = str.indexOf( kernel ) ) ){
                section[1] = str.length() - j + 1;
                return true;
            }
        }
        
	return false;
    }
    
    // method that shifts table
    public void shift( int index, int[] section, CCSTable.line_t line ) {
        char[][] table = m_table.getTable();
        String str;
        
        switch( line ) {
            case ROW:
                // jump first line
                for( int i=index ; i > 0 ; --i ) {
                    for( int j = section[0] ; j < section[0] + section[1] ; ++j ) {
                        table[i][j] = table[i-1][j];
                    }
                }
                
                m_table.setTable( table );
                m_table.fillGaps( 0, section[0], section[1], line );
            break;
            case COLUMN:
                str = String.copyValueOf( m_table.getColumn(index) );
                String replacement = str.substring(0, section[0]);
                int sec = section[0] + section[1] - 1;
                for( int i= replacement.length() -1 ; i >= 0 ; --i )
                    table[ sec-- ][ index ] = replacement.charAt(i);
                    
                m_table.setTable( table );
                m_table.fillGaps( index, 0, section[1] - section[0], line );
            break;
            default:
                System.err.println( "invalid line type" );
            break;
        }
    }
}
