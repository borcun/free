import java.util.Scanner;

// CCSGame class
public class CCSGame {
    // CCSRule class
    private static class CCSRule {
	public static final int MAX_MOVE = 10;
	public static final int SCORE = 40;
	public static final int MAX_SHUFFLE = 50;
	public static final int MIN_SEQUENCE = 3;
    }

    private CCSTable m_table;
    private int m_score;
    private int m_move;
    private boolean m_is_finish;
    // game status
    private enum Status {
	WIN, LOST, CONTINUE
    };

    // default constructor
    public CCSGame() {
	m_is_finish = false;
    }

    // method that starts game
    public void start( int row, int col, char[] icons ) {
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
		System.out.print("Enter a move : ");
		String move = scanner.next();
		String[] coordinate = move.split(",");
 
		// if table isn't steady, shuffle it until shuffle reaches maximum shuffle
		update( Integer.parseInt( coordinate[0] ), 
			Integer.parseInt( coordinate[1] ), 
			Integer.parseInt( coordinate[2] ), 
			Integer.parseInt( coordinate[3] ) );

		break;
	    } // end of switch
	} // end of while

	return;
    }

    // method that checks whether the game continues
    private Status check() {
	if( CCSRule.MAX_MOVE > m_move && m_score >= CCSRule.SCORE )
	    return Status.WIN;
	else if( CCSRule.MAX_MOVE == m_move && m_score < CCSRule.SCORE )
	    return Status.LOST;
	else
	    return Status.CONTINUE;
    }

    // method that updates table
    private void update( int _row, int _col, int row, int col ) {
	char[][] table = m_table.getTable();
	int shuffle = 0;
	boolean is_ok = false;

        m_table.print();
        
	do {
	    char temp = table[ _row ][ _col ];
	    table[ _row ][ _col ] = table[ row ][ col ];
	    table[ row ][ col ] = temp;

            m_table.setTable( table );
            m_table.print();

	    // check next operation, 
	    /*
	    for( int i=0 ; i < table.length ; ++i ) {
		int start, length;

		if( find( m_table.getRow( i ), start, length ) ) {
		    // perform
		}
	    } */

	    m_table.shuffle();	    
	    ++shuffle;
	} while( ++shuffle != CCSRule.MAX_SHUFFLE );

	++m_move;
	m_table.setTable( table );
	
	return;
    }

    // method that finds sequence
    private boolean find( char[] line, int start, int length ) {
	return false;
    }
}
