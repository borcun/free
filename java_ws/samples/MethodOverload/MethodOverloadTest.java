
public class MethodOverloadTest 
{
	public static void main(String[] args) 
	{
		MethodOverload method_overload = new MethodOverload();
		int result1;
		double result2;
		
		result1 = method_overload.pow(3, 4);
		result2 = method_overload.pow(4.0, 2);
		
		System.out.printf("result1 : %d\nresult2 : %.2f\n", result1, result2);
	}
}
