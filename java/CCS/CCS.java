package ccs;

import java.util.Scanner;

public class CCS {
    public static void main( String[] args ) {
	CCSGame ccs = new CCSGame();
	Scanner scanner = new Scanner( System.in );
	char[] icons = { 'a', '*', '-', '8' , '1'};
	int row, col, move, score;

	System.out.print( "Enter table row : ");
	row = scanner.nextInt();
	System.out.print( "Enter table column : ");
	col = scanner.nextInt();
        System.out.print( "Enter move count : " );
	move = scanner.nextInt();
        System.out.print( "Enter score : " );
	score = scanner.nextInt();
	
	ccs.start( row, col, icons, move, score );
	ccs.play();
    }
}
