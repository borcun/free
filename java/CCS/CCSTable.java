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

    private int m_row;
    private int m_col;
    private char[][] m_table;
    private IconSet m_iconset;

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
	return;
    }

    // method that sets CCS table with new table
    public void setTable( char[][] table ) {
	m_table = table;
	return;
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

    // method that shuffles table
    public void shuffle() {
	Random random = new Random();
	for( int i=0 ; i < m_row ; ++i ) {
	    char[] row = getRow( i );
	    
	    // Fisher-Yates shuffle method
	    for( int j=1 ; j < row.length ; ++j ) {
		int index = random.nextInt( row.length - 1 );
		char ch = row[index];
		row[index] = row[i];
		row[i] = ch;
	    }
	}

	for( int i=0 ; i < m_col ; ++i ) {
	    char[] col = getColumn( i );
	    
	    // Fisher-Yates shuffle method
	    for( int j=1 ; j < col.length ; ++j ) {
		int index = random.nextInt( col.length - 1 );
		char ch = col[index];
		col[index] = col[i];
		col[i] = ch;
	    }

	    // columns aren't reference as rows, so shuffled 
	    // columns have to be printed into table
	    for( int j=0 ; j < m_table.length ; ++j )
		m_table[j][i] = col[j];
	}

	return;
    }

    // method that print CCS table
    public void print() {
	for( char[] row : m_table ) {
            System.out.print("| ");
	    for( char col : row ) {
		System.out.printf( "%c ", col );
	    }
            
	    System.out.printf("%s\n", "|");
	}

        System.out.print("\n\n");
 
        return;
    }
}
