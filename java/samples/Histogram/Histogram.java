import java.util.Random;

public class Histogram 
{
	public static void display(int arr[]) 
	{
		for(int i=0 ; i < arr.length ; ++i) {
			System.out.print(arr[i]);
			
			for(int j=0 ; j < arr[i] ; ++j) {
				System.out.print("* ");
			}

			System.out.println();

		}
		
	}
	
	public static void main(String[] args) 
	{
		int arr[] = new int[10];
		Random random = new Random();
		
		for(int i=0 ; i < arr.length ; ++i)
			arr[i] = random.nextInt(10);
		
		display(arr);
	}
}
