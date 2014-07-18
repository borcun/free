
public class Arithmetic 
{
	public static void main(String args[])
	{
		int toplam,
			fark,
			carpim,
			mod;
		
		double bolum;
		
		int sayi1; 
		int sayi2 = 10;
		double sayi3 = 32.3;
		
		sayi1 = 32;
		
		toplam = sayi1 + sayi2;
		System.out.printf("Toplam : %d\n", toplam);

		fark = sayi1 - sayi2;
		System.out.printf("Fark : %d\n", fark);
		
		carpim = sayi1 * sayi2;
		System.out.printf("Carpim : %d\n", carpim);
		
		bolum = sayi3 / sayi2;
		System.out.printf("Bolum : %f\n", bolum);
		
		mod = sayi1 % sayi2;
		System.out.printf("Mod : %d\n", mod);
	}
}
