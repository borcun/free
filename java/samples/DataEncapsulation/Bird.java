
public class Bird 
{
	private String m_name;
	private String m_environment;
	private int m_age;

	// default constructor
	public Bird()
	{

	}

	// constructor
	public Bird(String name)
	{
		m_name = name;
	}

	// constructor
	public Bird(String name, String environment)
	{
		m_name = name;
		m_environment = environment;
	}

	// method that sets bird name
	public void setName(String name)
	{
		m_name = name;
	}

	// method that sets bird environment
	public void setEnvironment(String environment)
	{
		m_environment = environment;
	}

	// method that gets bird age
	public void setAge(int age)
	{
		m_age = age;
	}

	// method that gets bird name
	public String getName()
	{
		return m_name;
	}

	// method that gets bird environment
	public String getEnvironment()
	{
		return m_environment;
	}

	// method that gets bird name
	public int getAge()
	{
		return m_age;
	}

	// method that prints bird attributes
	public void printBirdAttributes()
	{
		System.out.printf("Bird Name : %s\nBird Environment : %s\nBird Age : %d\n",
						  m_name, m_environment, m_age);
	}
}
