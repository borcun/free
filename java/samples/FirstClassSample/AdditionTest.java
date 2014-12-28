import java.util.Scanner;

public class AdditionTest {

	public static void main(String[] args) 
	{
		Scanner input = new Scanner(System.in);
		Addition addition = new Addition();
		
		int num1,
			num2;
		
		String name;
		
		System.out.print("Ilk sayiyi giriniz : ");
		num1 = input.nextInt();
		
		System.out.print("\nIkinci sayiyi giriniz : ");
		num2 = input.nextInt();
		
		addition.getAddition(num1, num2);
	}

}
