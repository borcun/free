import java.util.Scanner;

public class NestedFor 
{
	public static void main(String args[])
	{
		Scanner input = new Scanner(System.in);
		int asteriks;
		
		System.out.print("Lutfen yildiz sayisini giriniz : ");
		asteriks = input.nextInt();
		
		for(int i=1 ; i <= asteriks ; ++i)
		{
			for(int j=0 ; j < i ; ++j)
			{
				System.out.print("*");
			}
			
			System.out.println();
		}
	}
}
