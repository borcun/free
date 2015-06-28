public class Magazine extends LibraryMaterial {
	public Magazine(int id) {
		super(id);
	}

	@Override
	public boolean isHireable() {
		return true;
	}
}
