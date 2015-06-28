/**
 * @file Date.java
 * @brief Date is a class which contains some operations related to date.
 * @author BEM
 *
 */

/// \brief class Date
public class Date {
	//! day
	private int m_day;
	//! month
	private int m_month;
	//! year
	private int m_year;
	
	/// \brief default constructor
	public Date() {
		// this is the default date
		this( 1, 1, 2000 );
	}
	
	/// \brief constructor
	/// @param day - day
	/// @param month - month
	/// @param year - year
	public Date( int day, int month, int year ) {
		setDay( day );
		setMonth( month );
		setYear( year );
	}
	
	/// \brief method that sets day of date
	/// @param day - day
	/// @return -
	public void setDay( int day ) {
		if( day <= 0 || day > 30 ) {
			System.err.printf( "invalid day : %d\n", day );
			m_day = 1;
		}
		else
			m_day = day;
		
		return;
	}
	
	/// \brief method that sets month of date
	/// @param month - month
	/// @return -
	public void setMonth( int month ) {
		if( month <= 0 || month > 12 ) {
			System.err.printf( "invalid month : %d\n", month );
			m_month = 1;
		}
		else
			m_month = month;
		
		return;
	}
	
	/// \brief method that sets year of date
	/// @param year - year
	/// @return -
	public void setYear( int year ) {
		if( year < 2000 ) {
			System.err.printf( "invalid year : %d\n", year );
			m_year = 2000;
		}
		else
			m_year = year;
		
		return;
	}

	/// \brief method that gets day of date
	/// @return day of date
	public int getDay() {
		return m_day;
	}

	/// \brief method that gets month of date
	/// @return month of date
	public int getMonth() {
		return m_month;
	}

	/// \brief method that gets year of date
	/// @return year of date
	public int getYear() {
		return m_year;
	}
	
	/// \brief method that compares two date
	/// @param date - Date instance
	/// @return difference of Date instance by day
	public int compare( Date date ) {
		int total1;
		int total2;
		
		total1 = date.getYear() * 365 + date.getMonth() * 30 + date.getDay();
		total2 = this.getYear() * 365 + this.getMonth() * 30 + this.getDay();
		
		return total2 - total1;
	}
	
	/// \brief method that finds next date
	/// @param day - day
	/// @return next date
	public Date after( int day ) {
		Date date = new Date();
		
		if( m_day + day > 30 ) {
			if( m_month + day / 30 > 12 ) {
				date.setYear( m_year + 1 );				
				date.setMonth( (m_month + day / 30) % 12 );
				date.setDay( m_day + ( day >= 365 ? day % 365 : day % 30 ) );
			}
			else {
				date.setYear( m_year );
				date.setMonth( m_month + day / 30 );
				date.setDay( m_day + day % 30 );
			}
		}
		else {
			date.setDay( m_day + day );
			date.setMonth( m_month );
			date.setYear( m_year );
		}
		
		return date;
	}

	/// \brief method that prints date as formatted
	/// @return formatted date
	public String toString() {
		return String.format("%d.%d.%d", m_day, m_month, m_year );
	}
}
