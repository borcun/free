Source: http://letshackarcadegames.com/?p=36

Mikroislemci Temelleri
======================
Bu yazida, mikroislemcinin nasil calistigini ve bir mikroislemci mimarisinin nasil olduguna bakacagiz.

Mikroislemci Nedir?
===================
Mikroislemciler, bilgisayar sisteminin beynidir. Programlanabilir genel amacli araclar olarak da bilinir.
Herhangi bir donanima baglanabilir, hafiza birimlerinden veri okuyarak donanim uzerinde belirli bir takim
gorevleri gerceklestirebilirler.

Hafiza birimlerinde saklanan verilerden kastimiz talimatlar (instructions) veya operasyon kodu (opcodes)
kavramlaridir. Bu kavramlar, islemcinin hangi talimati gerceklestirecegini belirtir. Talimatlar, tipik
olarak bazi kalici hafiza birimlerinde (ROM ve hdd) saklanirlar.

Ayni dizayn konseptine sahip olmalarina ragmen farkli mimariye sahip bir cok mikroislemci mevcuttur. Bu
farkli mimariler genelde su bilesenleri icerir: control unit, arithmetic and logic unit, register, system
bus.

http://letshackarcadegames.com/wp-content/uploads/2015/02/cpu_block_diagram.png

Control Unit
============
Adindan da anlasilacagi gibi kontrol birimi, islemcinin olaylari kontrol etme gorevine sahip oldugu
birimdir. Talimatlari hafizadan alma, alinan talimatlari aciklama ve talimatin gerceklenebilmesi icin
islemcinin farkli bilesenlerine sinyal gonderme gorevleri bu birim ile yapilir. Bu birimi, bir insaat
sirketinin yoneticisi olarak dusunebilirsiniz, yonetici insaat planlarini alir, genel gorevleri olan
ve bu gorevleri nasil icra edecegini bilen farkli bir kac insaat takimina planlari aktarir ve denetler.

Kontrol birim, sabit frekansta dusuk ve yuksek voltaj degerleri ureten ve bu sayede islemciye diger 
bilesenlerini kontrol ve senkronize etme sansi taniyan, titreyen bir saat (oscillating clock) araciligi 
ile surulur. Bu titreyen saatin hizi MHz(megahertz) veya GHz(gigahertz) birimleri ile aciklanir. Bu
birimler, insanlar islemci veya cekirdek hizlarindan bahsederken kullandiklari birimlerdir.

Arithmetic and Logic Unit
=========================
Aritmetik ve Mantik Birimi, kisaca ALU, matematiksel ve mantiksal islemlerin yapilmasindan sorumlu olan
birimdir. Bir seylerin toplamina, farkina, carpimina veya bolumune ihtiyac oldugu zaman, ALU devreye 
girer. Kullanicinin siradan matematiksel islemlerine ek olarak, ALU ayni zamanda mantiksal islemler icin
de kullanilir. Ornegin; boolean karsilastirmalari, shift (kaydirma) islemleri gibi.

Register
========
Bir an icin yazilim ile bilgisayar hafiza arasindaki iliskiyi dusunun. Bilgisayar uzerinde calisan
yazilimin yararli bir seyler yapabilmek adina, butun matematiksel islemler, zamanlama mekanizmalari,
daha sonra kullanilmak uzere kullanici girdilerini kayit etme islemleri icin periodik olarak RAM'dan 
datalar alinir veya RAM uzerinde saklanir.

Kaydediciler, kisa zamanli veri saklayan, mikroislemcideki kucuk RAM parcalari olarak dusunulebilir.
Programlama terimi olarak aciklamak gerekirse kaydediciler, mikroislemcinin talimat argumanlarinin
saklandigi, zamani geldigi anda uygun adresteki verilerin alip islenecegi kayit alanlari olarak
tanimlanabilir. Bir islemci mimarisinde genelde 4 adet kaydedici vardir: program counter register,
flag register, accumulator register ve general purpose register.

Program Counter Register
------------------------
Program Sayac Kaydedicisi, butun mikroislemcilerde ortak paylasilan kaydedicidir. Her zaman program
sayaci gorevini yerine getirmek icin kullanilmayabilirler fakat genelde bu islevi gerceklestirirler.
Program sayacinin amaci, programin duzenli bir sekilde calisabilmesi icin bir sonraki talimatin adresini
saklamaktir. Kontrol birimi bu talimati alip, isleyip, calistirdiktan sonra, program sayaci bir artar
ve kontrol birimine bir sonraki talimati calistirabilmesi icin olanak sunar.

Flag Register
-------------
Bayrak Kaydedicisi, mimariden mimariye farklilik gosterebilir ama genel itibariyle program sayaci gibi
kullanimi mimariler arasinda sabittir. Bayrak kaydedicisi, onceden calistirilmis talimatlar hakkinda
boolean bilgi saklayan kaydedicidir. Bayrak kaydedicisinin her biti true ve false durumuna karsilik
gelir. Ornegin; Hafizadaki degeri devamli olarak 1 azaltan fakat 0 oldugu anda durmasini istedigim bir
program yaziyorum. En olasi yol, degerin veya bayrak kaydedicisinin bitinin 0 olup olmadigini kontrol 
etmektir. Eger 0 biti 1 olarak sonlanmissa, bu daha onceki islemin sonucunun 0 oldugu anlamina gelir,
eger 0 biti 0 olarak sonlanmissa, bu da bir onceki islemin sonucunun 1 oldugunu gosterir.

Net olarak anlasilmadiysa, endise etmeyin cunku ilerleyen zamanlarda bayrak kaydedicisinin kullanimina
daha detayli olarak deginecegiz ama suan icin kaydedicinin gorevinin, bir onceki islemin sonucu hakkinda
basit olarak true veya false cevaplari saklamak icin kullanildigini bilmemiz yeterlidir.

Accumulator Register
---------------------
Hafiza kaydedicileri, adindan da anlasilacagi gibi islenen talimatin sonuclarinin saklandigi kaydediciler
dir. Daha genel tabirle, bir islemin sonuclarinin otomatik olarak saklandigi kaydedicilerdir. Butun
islemci mimarileri en azindan bir tane hafiza kaydedicisi icerir.

Genel orneklerde, hafiza kaydedicileri, matematiksel islemlerin sonuclarinin sakladigi gosterilir. Ornegin;
1 ile 2 sayisi toplanirsa, hafiza kaydedicisi 3 sayisini saklar.

General Purpose Register
------------------------
Genel amacli kaydediciler, ne amac icin olursa olsun programcinin uygun gordugu verileri saklamak icin
kullanilir. Islem sonucu olusan degerleri kayit etmezler, sadece kendi iclerinde saklanan degerler yerine
baska degerler yazilana kadar saklarlar.

Register Width
==============
Islemci kapasitesine gore (8, 16, 32 ve 64) kaydedicilerin genisligi degisebilir. Bazen oyunlar hakkinda 
yanlis olarak bilinen bir yargi vardir: 8-bit oyun sistemleri 8-bittir cunku 8-bit renk derinligi vardir.
Oysaki 8-bit sistemlerin 8-bit olmasi genel amacli kaydedicilerinin 8 bit olmasi ve en fazla 8 adet bit
saklayabilimesinden dolayidir.

Kaydedici genisligi, ozellikle program sayacinin genisligi, bir islemcinin rahatlikla kullanabilecegi
maximum hafiza miktarini belirler. Ornegin, 16-bit bir program sayaci rahatlikla 65.536 byte alana
erisebilir.

System Bus
==========
Tipki bir sehir otobusunun insanlari bir yerden baska yere tasimakla sorumlu oldugu gibi sistem yolu da
farkli islemci bilesenleri arasinda bilgi transferi yapmakla sorumludur. Mikroislemcilerde genel olarak 
3 adet sistem yolu mevcuttur: control bus, address bus ve data bus.

Control Bus
-----------
Kontrol yolu, ALU ile veri yolu arasinda kaydedici degerlerinin transferi ve bir takim islemleri gercekles
tirebilmek icin islemcinin farkli bilesenleri arasinda sinyal gonderme islemlerinden sorumlu olan yoldur.

Address Bus
-----------
Birisi ROM'dan, RAM'den veya harici bir cihazdan bir veriye erismek istedigi zaman, baska birisinin bu kisi
ye adres yolunu soylemesi gerekmektedir. Hafiza adresleri onaltilik tabanda tutulur ve birer birer artarlar.
Bu adresler, islemci icin uygun olan toplam hafizanin 1 byte'ini (8 bitini) temsil ederler.

Adres yolu, islemcinin data yolu uzerinden kendisine bir adres dondurmesi icin hafiza adresine bir sinyal 
gondermesine izin verir.

Data Bus
--------
Veri yolu, islemcinin kaydedicileri, harici cihazlar ve hafiza arasinda veri transferine izin veren yoldur.