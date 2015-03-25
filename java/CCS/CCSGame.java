package ccs;

import java.util.Scanner;

// CCSGame class
public class CCSGame {
    // CCSRule class
    private static class CCSRule {
	public static final int MAX_MOVE = 10;
	public static final int SCORE = 40;
	public static final int MAX_SHUFFLE = 3;
	public static final int MIN_SEQUENCE = 3;
    }

    private CCSTable m_table;
    private int m_score;
    private int m_move;
    private boolean m_is_finish;
    // game status
    private enum status_t {
	WIN, LOST, CONTINUE
    };
    // line type
    private enum line_t {
      ROW, COLUMN  
    };

    // default constructor
    public CCSGame() {
        System.out.println( "Welcome to Candy Crush Saga\n" );
	m_is_finish = false;
    }

    // method that starts game
    public void start( int row, int col, char[] icons ) {
        System.out.println( "\nStarting Game" );
	m_table = new CCSTable( row, col );
	m_table.create( icons );
	m_score = 0;
	m_move = 0;

	return;
    }

    public void play() {
	Scanner scanner = new Scanner( System.in );
	boolean is_continue = true;
	
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
                System.out.printf( "Move : %d, Score : %d\n", m_move, m_score );
		System.out.print("Enter a move : ");
		String move = scanner.next();
		String[] coordinate = move.split(",");
 
		// if table isn't steady, shuffle it until shuffle reaches maximum shuffle
		update( Integer.parseInt( coordinate[0] ), 
			Integer.parseInt( coordinate[1] ), 
			Integer.parseInt( coordinate[2] ), 
			Integer.parseInt( coordinate[3] ) );

                ++m_move;
		break;
	    } // end of switch
	} // end of while

	return;
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
        boolean is_operated = false;
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

                is_operated = false;

                // find operation starts from last line, because
                // any operation which performs in last line effects top lines
                for( int i = table.length - 1 ; i >= 0 ; --i ) {
                    int[] section = new int[2];

                    while( find( m_table.getRow( i ), section ) && !is_operated ) {
                        String str = String.copyValueOf(m_table.getRow( i ));
                        System.out.printf( "%d. row, from %d to %d : %s\n\n", i, section[0], section[1], str.substring(section[0], section[1]) );
                        shift( i, section, line_t.ROW );
                        m_table.print();
                        is_operated = true;
                        m_score += section[1] - section[0];
                        // start again
                        i = table.length - 1;
                    }
                }

                is_operated = false;

                for( int i = table.length - 1 ; i >= 0 ; --i ) {
                    for( int j=0 ; j < table[i].length ; ++j ) {
                        int[] section = new int[2];

                        while( find( m_table.getColumn( i ), section ) && !is_operated ) {
                            String str = String.copyValueOf(m_table.getColumn( i ));
                            System.out.printf( "%d. column, from %d to %d : %s\n\n", i, section[0], section[1], str.substring(section[0], section[1]) );
                            shift( i, section, line_t.COLUMN );
                            m_table.print();
                            m_score += section[1] - section[0];
                            is_operated = true;
                        }
                    }
                }

                if( !is_operated ) {
                    if( ++shuffle != CCSRule.MAX_SHUFFLE ) {
                        m_table.shuffle();
                        can_continue = true;
                    }
                    else
                        can_continue = false;
                }
                else
                    can_continue = is_operated;
            }
	} while( shuffle != CCSRule.MAX_SHUFFLE && !is_operated );
	
	return;
    }

    // method that finds sequence
    private boolean find( char[] line, int[] section ) {
        final int sequence = line.length;
        String str = String.valueOf(line);
                
        for( int i = sequence ; i >= CCSRule.MIN_SEQUENCE ; --i ) {
            for( int j = 0 ; j < sequence - i + 1 ; ++j ) {
                String substr = str.substring(j, i+j);
                
                if( substr.charAt(0) != ' ' ) {
                    char ch = substr.charAt(0);
                    boolean same = true;
                    for( int k=1 ; k < substr.length() ; ++k ) {
                        if( ch != substr.charAt(k) ) {
                            same = false;
                            break;
                        }
                    }

                    if( same ) {
                        section[0] = j;
                        section[1] = j+i;

                        return true;
                    }
                }
            }
        }
        
	return false;
    }
    
    // method that shifts table
    public void shift( int index, int[] section, line_t line ) {
        char[][] table = m_table.getTable();

        switch( line ) {
            case ROW:
                for( int i=section[0] ; i < section[1] ; ++i ) {
                    table[ index ][ i ] = ' ';
                }

                m_table.fillGaps();
                m_table.setTable( table );
            break;
            case COLUMN:
                for( int i=section[0] ; i < section[1] ; ++i ) {
                    table[ i ][ index ] = ' ';
                }
        
                m_table.fillGaps();
                m_table.setTable( table );
            break;
            default:
                System.err.println( "invalid line type" );
            break;
        }
                    
        return;
    }
}
