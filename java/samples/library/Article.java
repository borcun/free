public class Article extends LibraryMaterial {
	public Article(int id) {
		super(id);
	}

	@Override
	public boolean isHireable() {
		return false;
	}

}
