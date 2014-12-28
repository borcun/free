import java.util.Scanner;

public class CalculatorTest 
{
	public static void main(String args[])
	{
		Calculator calculator = new Calculator();
		Scanner input = new Scanner(System.in);
		int number1, number2;
		int counter = 0;
		int choice;
		
		while(counter < 10) {
			System.out.print("Please, enter first number : ");
			number1 = input.nextInt();
			calculator.setNumber1(number1);
			
			System.out.print("Please, enter second number : ");
			number2 = input.nextInt();
			calculator.setNumber2(number2);
			
			System.out.print("Please, enter operation (add: 1, sub: 2, div: 3, mult: 4) : ");
			choice = input.nextInt();			
			
			if(1 == choice) {
				System.out.printf("Sum : %d\n", calculator.add());
			}
			// sub
			if(2 == choice) {
				System.out.printf("Sub : %d\n", calculator.sub());
			}
			// div
			if(3 == choice) {
				System.out.printf("Div : %d\n", calculator.divide());
			}
			// mult
			if(4 == choice) {
				System.out.printf("Mult : %d\n", calculator.mult());
			}
			
			++counter;
		} // end while
		
	}
}
