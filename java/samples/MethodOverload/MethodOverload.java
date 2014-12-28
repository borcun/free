public class MethodOverload 
{
	public int pow(int base, int power)
	{
		int res = 1;

		for(int i=0 ; i < power ; ++i) {
			res *= base;
		}

		return res;
	}

	public double pow(double base, int power)
	{
		double res = 1.0;

		for(int i=0 ; i < power ; ++i) {
			res *= base;
		}

		return res;
	}
}
