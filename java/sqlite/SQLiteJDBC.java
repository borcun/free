
import java.sql.*;

public class SQLiteJDBC
{
  public static void main( String args[] )
  {
    Connection c = null;
    Statement statement = null;
    
    try {
      Class.forName("org.sqlite.JDBC");
      c = DriverManager.getConnection("jdbc:sqlite:test.db");
    } catch ( Exception e ) {
      System.err.println( e.getClass().getName() + ": " + e.getMessage() );
      System.exit(0);
    }
    
    System.out.println("Opened database successfully");
    
    try {
		statement = c.createStatement();
		statement.executeUpdate("create table Test (id integer, name varchar(16))");
		statement.executeUpdate("insert into Test values (1, 'burak')");
		statement.executeQuery("select * from Test");
	    statement.close();
	    c.close();
	    
	    System.out.println("Closed database successfully");
	    
    } catch (SQLException e) {
		System.out.println(e.getMessage());
	}
    
  }
}
