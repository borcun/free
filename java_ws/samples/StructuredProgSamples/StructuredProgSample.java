import java.util.Scanner;

public class Sample 
{
	public static void main(String[] args) 
	{
		Scanner input = new Scanner(System.in);
		int number;
		int choice = 0;
		
		System.out.print("Lutfen bir sayi giriniz : ");
		number = input.nextInt();
		
		if(choice == 0)
		{
			do
			{
				System.out.printf("%d ", number);
				++number;
			} while(number < 15);
		}
		else if(choice == 1) {
			int i=0;
			
			while(i < number) 
			{
				System.out.printf("%.1f ", Math.pow(i, 2));
				
				++i;
			}
		}
		else if(choice == 2) 
		{
			for(int i=0 ; i < number ; ++i) {
				if(i % 3 == 0 && i != 0) {
					System.out.print("* ");
				}
				else if(i == 10) {
					break;
				}
				else {
					System.out.printf("%d ", i);
				}
			}
			
			System.out.println();
		}
		else if(choice == 3) 
		{
			switch(number % 2)
			{
				case 0:
					System.out.printf("%d sayisi cifttir.\n", number);
				break;
				case 1:
					System.out.printf("%d sayisi tektir.\n", number);
				break;
				default:
					System.out.println("Gecersiz sayi");
				break;
			}
		}
		else if(choice == 4)
		{
			if(number % 2 == 0 && number % 3 != 0 && number % 5 != 0) {
				System.out.printf("%d sayisi ikinin katidir.\n", number);
			}
			else if(number % 3 == 0 && number % 5 != 0) {
				System.out.printf("%d sayisi ucun katidir.\n", number);
			}
			else if(number % 5 == 0) {
				System.out.printf("%d sayisi besin katidir.\n", number);
			}
			else {
				System.out.println("Gecersiz sayi");			
			}	
		}
	}

}
