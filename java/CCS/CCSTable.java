import java.util.Random;

// CCS Table class
public class CCSTable {
    private class IconSet {
	private String m_icons;
    
	// default constructor
	public IconSet() {
	    m_icons = "";
	}
	
	// method that adds icon to icon list
	public void addIcon( char icon ) {
	    m_icons += icon;
	    return;
	}
	
	public char[] getIcons() {
	    return m_icons.toCharArray();
	}

	public char getRandomIcon() {
	    return m_icons.charAt( (int)(Math.random() * m_icons.length()) );
	}
    } // end of IconSet class

    private final int m_row;
    private final int m_col;
    private char[][] m_table;
    private final IconSet m_iconset;
    // line type
    public static enum line_t {
      ROW, COLUMN  
    };

    // constructor
    public CCSTable( int row, int col ) {
	m_row = row;
	m_col = col;
	m_table = new char[ m_row ][ m_col ];
	m_iconset = new IconSet();
    }

    // method that creates table
    public void create( char[] icons ) {
	for( char icon : icons )
	    m_iconset.addIcon( icon );

	for( int i=0 ; i < m_table.length ; ++i ) {
	    for( int j=0 ; j < m_table[i].length ; ++j ) {
		m_table[i][j] = m_iconset.getRandomIcon();
	    }
	}
    }

    // method that sets CCS table with new table
    public void setTable( char[][] table ) {
	m_table = table;
    }

    // method that gets CCS table
    public char[][] getTable() {
	return m_table;
    }

    // method that gets row of CCS table
    public char[] getRow( int row ) {
	if( row < 0 || row >= m_row ) {
	    System.err.println( "invalid row index" );
	    return null;
	}
	return m_table[ row ];
    }

    // method that gets column of CCS table
    public char[] getColumn( int col ) {
	if( col < 0 || col >= m_col ) {
	    System.err.println( "invalid column index" );
	    return null;
	}

	char column[] = new char[ m_row ];

	for( int i=0 ; i < column.length ; ++i )
	    column[i] = m_table[i][col];
	
	return column;
    }
    
    // method that gets row count of table
    public int row() {
        return m_row;
    }

        // method that gets row count of table
    public int column() {
        return m_col;
    }

    // method that shuffles table
    public void shuffle() {
	Random random = new Random();
	for( int i=0 ; i < m_row ; ++i ) {
	    char[] row = getRow( i );
	    
	    // Fisher-Yates shuffle method
	    for( int j = row.length - 1 ; j > 0 ; --j ) {
		int index = random.nextInt( j + 1 );
		char ch = row[index];
		row[index] = row[j];
		row[j] = ch;
	    }
	}

	for( int i=0 ; i < m_col ; ++i ) {
	    char[] col = getColumn( i );
	    
	    // Fisher-Yates shuffle method
	    for( int j = col.length - 1 ; j > 0 ; --j ) {
		int index = random.nextInt( j + 1 );
		char ch = col[index];
		col[index] = col[j];
		col[j] = ch;
	    }

	    // columns aren't reference as rows, so shuffled 
	    // columns have to be printed into table
	    for( int j=0 ; j < m_table.length ; ++j )
		m_table[j][i] = col[j];
	}
    }

    // method that fills space with random character
    public void fillGaps( int index, int start, int length, line_t line ) {
        switch( line ) {
            case ROW:
                for( int i=start ; i < length ; ++i ) {
                    m_table[ index ][ i ] = m_iconset.getRandomIcon();
                }
            break;
            case COLUMN:
                for( int i=start ; i < length ; ++i ) {
                    m_table[ i ][ index ] = m_iconset.getRandomIcon();
                }
            break;
            default:
                System.err.println( "invalid line type" );
            break;
        } // end of switch
    }
    
    // method that print CCS table
    public void print() {
	for( char[] row : m_table ) {
            System.out.print("| ");
	    for( char col : row ) {
		System.out.print( col + " " );
	    }
            
	    System.out.print("|\n");
	}

        System.out.print("\n\n");
    }
}
