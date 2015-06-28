
public abstract class Person {
	private int m_id;
	private String m_name;
	private String m_surname;
	
	/// \brief default constructor
	public Person() {
		this( -1, "", "" );
	}
	
	/// \brief constructor
	/// @param id - user id
	/// @param name - user name
	/// @param surname - user surname
	public Person( int id, String name, String surname ) {
		setID( id );
		setName( name );
		setSurname( surname );
	}
	
	/// \brief method that sets user id
	/// @param id - user id
	/// @return
	public void setID( int id ) {
		m_id = id;
		return;
	}
	
	/// \brief method that sets user name
	/// @param name - user name
	/// @return
	public void setName( String name ) {
		m_name = name;
		return;
	}
	
	/// \brief method that sets user surname
	/// @param surname - user surname
	/// @return
	public void setSurname( String surname ) {
		m_surname = surname;
		return;
	}
	
	/// \brief method that gets user id
	/// @return user id 
	public int getID() {
		return m_id;
	}
	
	/// \brief method that gets user name
	/// @return user name 
	public String getName() {
		return m_name;
	}
	
	/// \brief method that gets user surname
	/// @return user surname 
	public String getSurame() {
		return m_surname;
	}
	 
	/// \brief method that returns person info
	/// @return person info
	public String toString() {
		switch( getJob() ) {
		case LIBRARIAN:
			return String.format( "[%d] %s %s (librarian)", m_id, m_name, m_surname );
		case USER:
			return String.format( "[%d] %s %s (user)", m_id, m_name, m_surname );
		}
		
		return String.format("%s", "invalid person info");
	}
	
	/// \brief method that gets job
	public abstract Utility.Job getJob();
}
