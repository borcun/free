public class NumberGenerator 
{
	public static void main(String args[])
	{
		int total = 0;
		int freq2 = 0;
		int freq3 = 0;
		int freq5 = 0;
		int freq7 = 0;
		int other = 0;
		int powNum;
	 	
		for(int i = 1 ; i < 101 ; ++i) {
			powNum = (int) Math.pow(i, 2);
			total += powNum;
			
			if(powNum % 2 == 0) {
				++freq2;
			}
			else if(powNum % 3 == 0) {
				++freq3;
			}
			else if(powNum % 5 == 0) {
				++freq5;
			}
			else if(powNum % 7 == 0) {
				++freq7;
			}
			else {
				++other;
			}
		} // end for
		
		System.out.printf("Total : %d, Average : %d\n", total, total / 100);
		System.out.printf("Ikiye bolunen sayilar : %d\n", freq2);
		System.out.printf("Uce bolunen sayilar : %d\n", freq3);
		System.out.printf("Bese bolunen sayilar : %d\n", freq5);
		System.out.printf("Yediye bolunen sayilar : %d\n", freq7);
		System.out.printf("2, 3, 5 ve 7 sayisina bolunmeyen sayilar: %d\n", other);

		return;
	}

}
