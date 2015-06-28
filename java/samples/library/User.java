public class User extends Person {
	/// \brief default constructor
	public User() {
		super();
	}
	
	/// \brief constructor
	/// @param id - user id
	/// @param name - user name
	/// @param surname - user surname
	public User( int id, String name, String surname ) {
		super( id, name, surname );
	}

	/// \brief method that gets user job
	/// @return user job
	@Override
	public Utility.Job getJob() {
		return Utility.Job.USER;
	}

}
