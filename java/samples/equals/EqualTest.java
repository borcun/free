public class EqualTest {
	public static void main( String[] args ) {
		String str1 = new String( "orcun" );
		String str2 = "orcun";

		if( str1 == "orcun" )
			System.out.println(" == operator is compared for str1" );
		else
			System.out.println(" == operator isn\'t compared for str1" );

		if( str2 == "orcun" )
			System.out.println(" == operator is compared for str2" );
		else
			System.out.println(" == operator isn\'t compared for str2" );
	}
}
