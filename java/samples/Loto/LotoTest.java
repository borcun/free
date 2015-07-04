
/// \brief class LotoTest
public class LotoTest {
	public static void main(String[] args) {
		Loto my_loto = new Loto();
		int[] my_numbers = new int[ my_loto.getColonLimit() ];
		
		my_numbers = my_loto.generate();
		
		for( int i=0 ; i < my_numbers.length ; ++i )
			System.out.printf( "%d ", my_numbers[ i ] );
	}

}
