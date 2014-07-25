public class ArraySample 
{
	public static void main(String[] args) 
	{
		char city[] = new char[6];
		
		city[0] = 'A';
		city[1] = 'N';
		city[2] = 'K';
		city[3] = 'A';
		city[4] = 'R';
		city[5] = 'A';
		
		System.out.printf("city arrayinin boyutu : %d\n", city.length);
		System.out.printf("city arrayinin hafizada kapladigi alan : %d byte\n", city.length * 2);
		
		
		for(int i=0 ; i < city.length ; ++i) {			
			if(i == 2) {
				city[i] = 'G';
			}
				
			System.out.printf("%c", city[i]);
		}
		
		System.out.println();
	}

}
