import java.util.Arrays;
import java.util.Random;

/// \brief class Loto
public class Loto {
	//! maximum number of loto
	private static final int MAX_NUM = 49;
	//! minimum number of loto
	private static final int MIN_NUM = 1;
	//! maximum colon count
	private static final int COLON_LIMIT = 6;
	
	/// \brief method that generate six numbers
	/// @return generated numbers
	public int[] generate() {
		int[] numbers = new int[ COLON_LIMIT ];
		Random random = new Random();
		int counter = 0;
		
		do {
			int number = random.nextInt( MAX_NUM ) + MIN_NUM;
			
			if( !duplicate( numbers, number ) ) {
				numbers[ counter ] = number;
				++counter;
			}
			
		} while( counter != COLON_LIMIT );
		
		Arrays.sort( numbers );
		
		return numbers;
	}
	
	/// \brief method that checks whether list contains element
	/// @return if the element is in list, return true. Otherwise, return false.
	public boolean duplicate( int[] list, int element ) {
		for( int i=0 ; i < list.length ; ++i ) {
			if( list[ i ] == element )
				return true;
		}
		
		return false;
	}
	
	/// \brief method that gets colon limit
	/// @return maximum colon limit
	public int getColonLimit() {
		return COLON_LIMIT;
	}
}
