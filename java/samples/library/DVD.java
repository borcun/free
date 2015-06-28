
public class DVD extends LibraryMaterial {
	public DVD( int id ) {
		super(id);
	}

	@Override
	public boolean isHireable() {
		return true;
	}
}
