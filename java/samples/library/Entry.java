
public class Entry {

	public static void main(String[] args) {
		LibraryMaterial[] libmat = new LibraryMaterial[4];
		
		libmat[0] = new Book(1);
		libmat[1] = new DVD(2);
		libmat[2] = new Magazine(3);
		libmat[3] = new Article(4);
		
		libmat[0].setName("HTPJ");
		libmat[0].setProducer("deitel");
		libmat[0].setProductionDate( new Date(10, 10, 2001) );
		libmat[0].setHiringTime(30);

		libmat[1].setName("Inglorious Bastards");
		libmat[1].setProducer("Tarantino");
		libmat[1].setProductionDate( new Date(5, 10, 2010) );
		libmat[1].setHiringTime(5);

		libmat[2].setName("Pardus");
		libmat[2].setProducer("TUBITAK");
		libmat[2].setProductionDate( new Date(15, 1, 2015) );
		libmat[2].setHiringTime(14);

		libmat[3].setName("Epipolar Geometry");
		libmat[3].setProducer("Z. Zhang");
		libmat[3].setProductionDate( new Date(17, 9, 2001) );
		libmat[3].setHiringTime(60);
		
		for( LibraryMaterial lm : libmat )
			System.out.println( lm );

	}
}
