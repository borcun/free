
public class ReverseNumber 
{
	public static void reverse(int num)
	{
		if(num < 10) {
			System.out.println(num);
			return;
		}
		
		System.out.printf("%d", num % 10);
		reverse(num / 10);
	}
	
	public static void main(String[] args) 
	{
		int number = 7613;
		
		reverse(number);
	}
}
