import java.util.Random;
import java.util.Scanner;

public class GuessNumber 
{
	public static void main(String args[])
	{
		Random random = new Random();
		Scanner scanner = new Scanner(System.in);
		
		int predicate;
		boolean isLoop = true;
		int step = 0;
		
		System.out.print("Lutfen sayi giriniz: ");
		predicate = scanner.nextInt();
		
		while(isLoop) {
			++step;
			
			if(predicate == random.nextInt(100)) {
				System.out.printf("%d tahminde buldum.\n", step);
				isLoop = false;
			}
		}
		
		return;
	}
}
