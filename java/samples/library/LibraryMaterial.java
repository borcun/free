/**
 * 
 * @author BEM
 *
 */

/// \brief class LibraryMaterial
public abstract class LibraryMaterial implements Hireable {
	private int m_id;
	private String m_name;
	private String m_producer;
	private Date m_production_date;
	private int m_hiring_time;
	
	/// \brief default constructor
	public LibraryMaterial( int id ) {
		if( id < 0 ) {
			System.err.println( "invalid id" );
			m_id = -1;
		}
		else
			m_id = id;
		
		m_name = "";
		m_producer = "";
		m_production_date = new Date();
		m_hiring_time = 0;
	}
	
	public void setName( String name ) {
		m_name = name;
		return;
	}

	public void setProducer( String producer ) {
		m_producer = producer;
		return;
	}

	public void setProductionDate( Date production_date ) {
		m_production_date = production_date;
		return;
	}

	public void setHiringTime( int hiring_time ) {
		if( isHireable() )
			m_hiring_time = hiring_time;
		return;
	}

	public int getID() {
		return m_id;
	}
	
	public String getName() {
		return m_name;
	}
	
	public String getProducer() {
		return m_producer;
	}
	
	public Date getProductionDate() {
		return m_production_date;
	}
	
	public int getHiringTime() {
		return m_hiring_time;
	}
	
	public String toString() {
		return String.format("[%d] %s, " + m_production_date + ", %s, %d", 
				m_id, m_name, m_producer, m_hiring_time);
	}
}
