import java.util.Scanner;

public class Calculator {
	public static void main( String[] args ) {
		Scanner scanner = new Scanner( System.in );
		char op;
		char choice = 'n';
		int number1, number2;

		do {
			System.out.print( "Please, enter operation : " );
			op = scanner.next().charAt( 0 );

			switch( op ) {
				// addition
			case '+' :
				System.out.print( "Please, enter first number : " );
				number1 = scanner.nextInt();
				System.out.print( "Please, enter second number : " );
				number2 = scanner.nextInt();

				System.out.printf( "%d + %d = %d\n", number1, number2, number1 + number2 );
				break;
				// substraction
			case '-' :
				System.out.print( "Please, enter first number : " );
				number1 = scanner.nextInt();
				System.out.print( "Please, enter second number : " );
				number2 = scanner.nextInt();

				System.out.printf( "%d - %d = %d\n", number1, number2, number1 - number2 );
				break;
				// multiplication
			case '*' :
				System.out.print( "Please, enter first number : " );
				number1 = scanner.nextInt();
				System.out.print( "Please, enter second number : " );
				number2 = scanner.nextInt();

				System.out.printf( "%d * %d = %d\n", number1, number2, number1 * number2 );
				break;
				// division
			case '/' :
				System.out.print( "Please, enter first number : " );
				number1 = scanner.nextInt();
				System.out.print( "Please, enter second number : " );
				number2 = scanner.nextInt();

				if( number2 != 0 )
					System.out.printf( "%d / %d = %.2f\n", number1, number2, (double)number1 / number2 );
				else
					System.err.println( "invalid divider" );
				break;
				// modulo
			case '%' :
				System.out.print( "Please, enter first number : " );
				number1 = scanner.nextInt();
				System.out.print( "Please, enter second number : " );
				number2 = scanner.nextInt();

				System.out.printf( "%d %% %d = %d\n", number1, number2, number1 % number2 );
				break;
				// invalid case
			default:
				System.err.println( "invalid operation type" );
				break;
			}
			
			System.out.print( "Continue? (y / n ) : " );
			choice = scanner.next().charAt( 0 );
		} while( choice == 'y' );
	}
}
