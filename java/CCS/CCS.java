import java.util.Scanner;

public class CCS {
    public static void main( String[] args ) {
	CCSGame ccs = new CCSGame();
	Scanner scanner = new Scanner( System.in );
	char[] icons = { 'a', '*', '-', '8' , '1'};
	int row = 0, col = 0;
        int move = 0, score = 0;

        do {
            System.out.print( "Enter table row : ");
            row = scanner.nextInt();
            System.out.print( "Enter table column : ");
            col = scanner.nextInt();

            if( row < 0 || col < 0 )
                System.err.println( "Please, enter positive value for row and column" );            
        } while( row <= 0 || col <= 0 );

        do {
            System.out.print( "Enter move count : " );
            move = scanner.nextInt();
            System.out.print( "Enter score : " );
            score = scanner.nextInt();

            if( move < 0 || score < 0 )
                System.err.println( "Please, enter positive value for move and score" );            
        } while( move <=0 || score <=0 );
        
	ccs.start( row, col, icons, move, score );
	ccs.play();
        
        scanner.close();
    }
}
