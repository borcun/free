[B]Mikroislemci Temelleri
======================[/B]
Bu yazida, mikroislemcinin nasil calistigini ve bir mikroislemci mimarisinin nasil olduguna bakacagiz.

[B]Mikroislemci Nedir?
===================[/B]
Mikroislemciler, bilgisayar sisteminin beynidir. Programlanabilir genel amacli araclar olarak da bilinirler.
Herhangi bir donanima baglanabilir, hafiza birimlerinden veri okuyarak donanim uzerinde belirli bir takim
gorevleri gerceklestirebilirler.

Hafiza birimlerinde saklanan verilerden kastimiz talimatlar (instructions) veya operasyon kodlari (opcodes)
kavramlaridir. Bu kavramlar, islemcinin hangi talimati gerceklestirecegini soyler. Talimatlar, tipik olarak 
bazi kalici hafiza birimlerinde (ROM ve hdd) saklanirlar.

Ayni dizayn konseptine sahip olmalarina ragmen farkli mimariye sahip bir cok mikroislemci mevcuttur. Bu
farkli mimariler genelde ortak olan su bilesenleri icerir: control unit, arithmetic and logic unit, register, 
system bus.

[IMG]http://i.hizliresim.com/6ZDgAN.png[/IMG]

[B]Control Unit
============[/B]
Adindan da anlasilacagi gibi kontrol birimi, islemcinin olaylari kontrol etme gorevine sahip oldugu birimdir. 
Talimatlari hafizadan alma, alinan talimatlari aciklama ve talimatlarin gerceklenebilmesi icin islemcinin 
farkli bilesenlerine sinyal gonderme gorevleri bu birim tarafindan yapilir. Bu birimi, bir insaat sirketinin 
yoneticisi olarak dusunebilirsiniz. Yonetici, insaat planlarini alir, belirli gorevleri olan ve gorevlerini 
nasil icra edecegini bilen farkli bir kac insaat takimina planlari aktarir ve sureci denetler.

Kontrol birimi, sabit frekansta dusuk ve yuksek voltaj degerleri ureten ve bu sayede islemciye diger bilesenlerini 
kontrol ve senkronize etme sansi taniyan, titreyen bir saat (oscillating clock) araciligi ile surulur. Bu titreyen 
saatin hizi MHz(megahertz) veya GHz(gigahertz) birimleri ile aciklanir. Bu birimler, insanlarin islemci veya 
cekirdek hizlarindan bahsederken kullandiklari birimlerdir.

[B]Arithmetic and Logic Unit
=========================[/B]
Aritmetik ve Mantik Birimi, kisaca ALU, matematiksel ve mantiksal islemlerin yapilmasindan sorumlu olan birimdir. 
Bir seylerin toplamina, farkina, carpimina veya bolumune ihtiyac oldugu zaman, ALU devreye girer. Kullanicinin 
siradan matematiksel islemlerine ek olarak, ALU ayni zamanda mantiksal islemler icin de kullanilir. Ornegin; 
boolean karsilastirmalari, shifting (bit kaydirma) islemleri gibi.

[B]Register
========[/B]
Bir an icin yazilim ile bilgisayar hafizasi arasindaki iliskiyi dusunun. Bilgisayar uzerinde calisan yazilimin 
yararli bir seyler yapabilmek adina, butun matematiksel islemler, zamanlama mekanizmalari ve daha sonra kullanilmak 
uzere kullanici girdilerini kayit etme islemleri icin periodik olarak RAM'dan datalar alinir veya RAM uzerinde 
saklanir.

Kaydediciler, kisa zamanli veri saklayan, mikroislemcideki kucuk RAM parcalari olarak dusunulebilir. Programlama 
terimi olarak aciklamak gerekirse kaydediciler, mikroislemcinin talimat argumanlarinin saklandigi, zamani geldigi 
anda uygun adresteki verilerin alip islenecegi kayit alanlari olarak tanimlanabilir. Bir islemci mimarisinde genelde 
4 adet kaydedici vardir: program counter register, flag register, accumulator register ve general purpose register.

[B]Program Counter Register
------------------------[/B]
Program Sayac Kaydedicisi, butun mikroislemcilerde ortak olan kaydedicidir. Her zaman program sayaci gorevini yerine 
getirmek icin kullanilmayabilirler fakat genelde bu islevi gerceklestirirler. Program sayacinin amaci, programin 
duzenli bir sekilde calisabilmesi icin bir sonraki talimatin adresini saklamaktir. Kontrol birimi bu talimati alip, 
isleyip, calistirdiktan sonra, program sayaci bir artar ve kontrol birimine bir sonraki talimati calistirabilmesi 
icin olanak sunar.

[B]Flag Register
-------------[/B]
Bayrak Kaydedicisi, mimariden mimariye farklilik gosterebilir ama genel itibariyle program sayaci gibi kullanimi 
mimariler arasinda sabittir. Bayrak kaydedicisi, onceden calistirilmis talimatlar hakkinda boolean bilgi saklayan 
kaydedicidir. Bayrak kaydedicisinin her biti true ve false durumuna karsilik gelir. Ornegin; hafizadaki degeri devamli 
olarak 1 azaltan fakat 0 oldugu anda durmasini istedigim bir program yaziyorum. En olasi yol, degerin veya bayrak 
kaydedicisinin bitinin 0 olup olmadigini kontrol etmektir. Eger 0 biti 1 olarak sonlanmissa, bu daha onceki islemin 
sonucunun 0 oldugu anlamina gelir, eger 0 biti 0 olarak sonlanmissa, bu da bir onceki islemin sonucunun 1 oldugunu 
gosterir.

Net olarak anlasilmadiysa, endise etmeyin cunku ilerleyen zamanlarda bayrak kaydedicisinin kullanimina daha detayli 
olarak deginecegiz ama suan icin kaydedicinin gorevinin, bir onceki islemin sonucu hakkinda basit olarak true veya 
false cevaplari saklamak icin kullanildigini bilmemiz yeterlidir.

[B]Accumulator Register
---------------------[/B]
Hafiza kaydedicileri, adindan da anlasilacagi gibi islenen talimatin sonuclarinin saklandigi kaydedicilerdir. Daha 
genel tabirle, bir islemin sonuclarinin otomatik olarak saklandigi kaydedicilerdir. Butun islemci mimarilerinde en 
azindan bir tane hafiza kaydedicisi bulunur.

Genel orneklerde, hafiza kaydedicileri, matematiksel islemlerin sonuclarinin sakladigi gosterilir. Ornegin; 1 ile 2 
sayisi toplanirsa, hafiza kaydedicisi 3 sayisini saklar.

[B]General Purpose Register
------------------------[/B]
Genel amacli kaydediciler, amac ne olursa olsun programcinin uygun gordugu verileri saklamak icin kullanilir. Islem 
sonucu olusan degerleri kayit etmezler, sadece kendi iclerinde saklanan degerler yerine baska degerler yazilana kadar 
eski degerleri saklarlar.

[B]Register Width
==============[/B]
Islemci kapasitesine gore (8, 16, 32 ve 64) kaydedicilerin genisligi degisebilir. Bazen oyunlar hakkinda yanlis olarak 
bilinen bir yargi vardir: 8-bit oyun sistemleri 8-bittir cunku 8-bit renk derinligi vardir. Oysaki 8-bit sistemlerin 
8-bit olmasi demek genel amacli kaydedicilerinin 8 bit olmasi ve en fazla 8 adet bit veri saklayabilimesinden dolayidir.

Kaydedici genisligi, ozellikle program sayacinin genisligi, bir islemcinin rahatlikla kullanabilecegi maximum hafiza 
miktarini belirler. Ornegin, 16-bit bir program sayaci rahatlikla 65.536 byte alana erisebilir.

[B]System Bus
==========[/B]
Tipki bir sehir otobusunun insanlari bir yerden baska yere tasimakla sorumlu oldugu gibi sistem yolu da farkli islemci 
bilesenleri arasinda bilgi transferi yapmakla sorumludur. Mikroislemcilerde genel olarak 3 adet sistem yolu mevcuttur: 
control bus, address bus ve data bus.

[B]Control Bus
-----------[B]
Kontrol yolu, ALU ile veri yolu arasinda kaydedici degerlerinin transferi ve bir takim islemleri gerceklestirebilmek 
icin islemcinin farkli bilesenleri arasinda sinyal gonderme islemlerinden sorumlu olan yoldur.

[B]Address Bus
-----------[/B]
Birisi ROM'dan, RAM'den veya harici bir cihazdan bir veriye erismek istedigi zaman, baska birisinin bu kisiye adres 
yolunu soylemesi gerekmektedir. Hafiza adresleri onaltilik tabanda tutulur ve birer birer artarlar. Bu adresler, islemci
icin uygun olan toplam hafizanin 1 byte'ini (8 bitini) temsil ederler.

Adres yolu, islemcinin data yolu uzerinden kendisine bir adres dondurmesi icin hafiza adresine bir sinyal gondermesine 
izin verir.

[B]Data Bus
--------[/B]
Veri yolu, islemcinin kaydedicileri, harici cihazlar ve hafiza arasindaki veri transferine izin veren yoldur.

Kaynak Source: http://letshackarcadegames.com/?p=36