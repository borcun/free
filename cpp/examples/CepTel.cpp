#include <iostream>
#include <string>

class CepTel {
public:
	CepTel( std::string marka, int fiyat, double agirlik, std::string renk = "siyah" ) {
		m_marka = marka;
		m_fiyat = fiyat;
		m_agirlik = agirlik;
		m_renk = renk;
	}

	void renkDegistir( std::string renk ) {
		m_renk = renk;
	}

	void fiyatDegistir( int fiyat ) {
		m_fiyat = fiyat;
	}

	void goster() {
		std::cout << "Marka   : " << m_marka << std::endl;
		std::cout << "Fiyat   : " << m_fiyat << std::endl;
		std::cout << "Agirlik : " << m_agirlik << std::endl;
		std::cout << "Renk    : " << m_renk << std::endl;
	}

private:
	std::string m_marka;
	int m_fiyat;
	double m_agirlik;
	std::string m_renk;
};

int main() {
	CepTel lg( "LG G2", 1200, 151.3, "kirmizi" );
	CepTel samsung( "Samsung S5", 1700, 145.7 );

	lg.goster();
	std::cout << std::endl;
	samsung.goster();

	lg.renkDegistir( "mavi" );
	lg.fiyatDegistir( 1149 );

	samsung.renkDegistir( "beyaz" );
	samsung.fiyatDegistir( 1649 );

	std::cout << std::endl;
	lg.goster();
	std::cout << std::endl;
	samsung.goster();

	return 0;
}
