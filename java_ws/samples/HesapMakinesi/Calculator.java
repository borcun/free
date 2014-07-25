public class Calculator 
{
	private int m_number1;
	private int m_number2;
	
	// default constructor
	public Calculator()
	{
		
	}
	
	// function that sets first number
	public void setNumber1(int number1)
	{
		m_number1 = number1;
	}

	// function that sets second number
	public void setNumber2(int number2)
	{
		m_number2 = number2;
	}

	// function that add two integers
	public int add() 
	{
		int res;
		
		res = m_number1 + m_number2;
		
		return res;
	}
	
	// function that sub two integers
	public int sub() 
	{
		int res;
		
		res = m_number1 - m_number2;
		
		return res;
	}
	
	// function that divide two integers
	public int divide() 
	{
		int res;
		
		res = m_number1 / m_number2;
		
		return res;
	}

	// function that add two integers
	public int mult() 
	{
		int res;
		
		res = m_number1 * m_number2;
		
		return res;
	}

}
