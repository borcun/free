
public class Book extends LibraryMaterial {
	public Book( int id ) {
		super(id);
	}

	@Override
	public boolean isHireable() {
		return true;
	}
}
