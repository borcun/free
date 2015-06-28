public class Librarier extends Person {
	/// \brief default constructor
	public Librarier() {
		super();
	}
	
	/// \brief constructor
	/// @param id - librarian id
	/// @param name - librarian name
	/// @param surname - librarian surname
	public Librarier( int id, String name, String surname ) {
		super( id, name, surname );
	}

	/// \brief method that gets librarian job
	/// @return librarian job
	@Override
	public Utility.Job getJob() {
		return Utility.Job.LIBRARIAN;
	}

}
