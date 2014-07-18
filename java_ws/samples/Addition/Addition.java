// ...
// ...

import java.util.Scanner;

public class Addition 
{
	// ...
	public static void main(String[] args) 
	{
		Scanner input = new Scanner(System.in);
		
		int sayi1, 
			sayi2, 
			toplam;
		
		System.out.print("Ilk sayiyi giriniz\n -> ");
		sayi1 = input.nextInt();
		
		System.out.print("Ikinici sayiyi giriniz\n -> ");
		sayi2 = input.nextInt();
		
		toplam = sayi1 + sayi2;
		
		System.out.printf("%d + %d = %d\n", sayi1, sayi2, toplam);
		
	} // ...

} // ...
