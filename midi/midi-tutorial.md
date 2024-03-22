[Orjinal Dokuman Adresi](https://www.cs.cmu.edu/~music/cmsip/readings/MIDI%20tutorial%20for%20programmers.html)

# MIDI Dersleri

Bu MIDI dersi sana, MIDI protokolu kullanan herhangi bir cihazi kontrol etmek icin MIDI dilini nasil\
kullanacagini anlaman konusunda yardimci olacaktir.

## MIDI Dersleri 1 - MIDI Mesajlari

MIDI dili, bir muzik parcasinin banttan calinmasi icin gercek zamanli bilgi gondermek amaciyla\
kullanilir.

Gercek zamanli terimi, bir mesajin, tam olarak hedef sentezleyici (donanimsal veya yazilimsal bir\
sentezleyici olabilir) tarafindan yorumlanmasi gereken anda gonderilmesi anlamina gelmektedir.

Bir muzigin banttan calinmasi icin gerekli olan bilgiyi gondermek icin farkli mesajlar tanimlanmistir.

Burada onemli olan nokta; MIDI dili sesin kendisini tanimlanamaz, ancak hedef sentezleyicideki sesi\
yaratmak icin gerekli olan komutlar serisini tanimlar.

MIDI mesajlari, zamanl-sirali bir veya daha fazla byte (8 bit) verisi olarak gonderilirler. Ilk byte\
**STATUS byte** verisidir, cogunlukla bu veriyi ek parametrelere sahip **DATA byte** verisi takip eder.\
Bir **STATUS byte** verisi 1'e ayarlanmis 7 adet bit'e, **DATA byte** verisi 0'a ayarlanmis 7 adet bit'e\
sahiptir.

**STATUS byte** verisi MIDI mesajinin tipini belirler. Onu takip eden **DATA byte** verisinin uzunlugu\
mesajin tipine baglidir.

Bazi sistem MIDI mesajlari disinda, **STATUS byte** verisi MIDI kanal sayisini icerir. Hexadecimal\
formatta ve 0'dan 15'e kadar numaralandirilmis 16 MIDI kanali vardir. Pratikte, muzisyenler ve\
yazilimlar, MIDI kanallarini 1'den 16'a kadar isimlendirirler, bu nedenle onlari hexadecimal olarak\
programladiginda aradaki 1 fark olacaktir. (kanal "1" "0" olarak, kanal "10" "9" olarak ve kanal "16"\
"F" olarak kodlanir.

Ayni MIDI kablosunda, 16 adete kadar enstrumani bagimsiz olarak kontrol etmek icin 16 adete kadar\
MIDI kanali olabilir.

### MIDI RUNNING STATUS (MIDI Calisma Statusu)

Bir MIDI mesaji okunurken, aslinda **STATUS byte** verisinin cikarilabilecegini bilmeniz gerekmektedir.\
(Ilk mesaj icerisinde gonderilen, mesajin tipini soyleyen **STATUS byte** verisi haric)

Boyle bir durumda, sadece **DATA byte** verisini iceren mesaj alabilirsiniz. Akabinde, **STATUS byte**\
verisinin, en son alinan **STATUS byte** verisi ile ayni oldugu kabul edilir.

Bu surec *MIDI RUNNING STATUS* olarak isimlendirilir. Ayni mesajin, uzun bir seri icerisinde gonderildigi\
durumlarda, veri transferini optimize etmek icin fayda saglamaktadir. Ornegin; pitch bend (zift kivrimi)\
veya crescendo volume curve (krisendo hacim egrisi).

*MIDI RUNNING STATUS* olayini, MIDI mesajlari urettiginiz zamanlarda da kullanabilirsiniz, ancak\
hedef synthesizer veya yazilimin bu durum bilgisini nasil aldigi ve onu iyi bir sekilde yorumladigi\
konularinda dikkatli ve emin olmalisiniz.

## MIDI Dersleri 2 - NOTE Mesajlari

Temel mesajlar **NOTE ON** ve **NOTE OFF** mesajlaridir.

**NOTE ON** mesaji, muzisyen muzik klavyesinin bir tusuna vurdugu anda gonderilir. Mesaj nota hizinin\
(tusa basildigi andaki notanin yogunlugu) yani sira pitch bilgisini de saglayan parametreler icer.

Bir sentezleyici bu mesaji aldiginda, ilgili notayi dogru pitch ve guc seviyeleri ile birlikte calmaya\
baslar.

**NOTE OFF** mesaji alindiginda, ilgili mesajdaki nota, sentezleyici tarafindan kapatilir.

### NOTE ON - OFF Mesajlari

Her *NOTE ON* mesajinin bir *NOTE OFF* karsiligi olmalidir, aksi takdirde nota sonsuza kadar calacaktir.\
Perkusyon enstrumanlari icin bir istisna vardir, bir perkusyon enstrumani sadece *NOTE ON* gonderebilir,\
cunku perkusyon notasi kendi kendine otomatik olarak durur. Ancak, her durumda *NOTE OFF* gondermek\
daha iyi bir pratiktir, cunku mesaji alan sentezleyici tarafindan nasil yorumlanabilecegine emin olamazsiniz.

**NOTE ON** mesaj yapisi su sekildedir:

 1. Status byte : 1001 CCCC
 2. Data byte 1 : 0PPP PPPP
 3. Data byte 2 : 0VVV VVVV
 
 - "CCCC", MIDI kanali (0'dan 15'e kadar)
 - "PPP PPPP", pitch degeri (0'dan 127'e kadar)
 - "VVV VVVV", velocity degeri (0'dan 127'e kadar)
 
Pitch degeri, calinacak notanin frekansini belirler. 0'dan 127'e kadar bir araliktadir, \
60 degeri ile *Middle C* notasi gosterilir.

![*Middle C*](https://www.cs.cmu.edu/~music/cmsip/readings/MIDI%20tutorial%20for%20programmers_files/midi-note-c60.jpg)

Notanin degeri, yarim adimlarda da gosterilir, yani nota C# 61, nota D 62...

Bir notayi, bir oktav yukari tasimak icin, pitch degerine 12 eklenir. MIDI kullanarak, oktav tasima,\
mevcut degeri sabit sayilar ekleyip cikarma yaparak kolay bir sekilde yapilir.

0'dan 127'e kadar giden MIDI notalarinin araliklarinin nasi degistigi konusunda dikkat olmak gerekir.\
Ornegin; degeri 96 olan bir notaya 4 oktav (+48) ekleyerek, toplamda 144'e ulasilir ki bu deger aralik\
disindadir ve 144-128 = 16 olacak sekilde truncate edilir, 16 da cok dusuk bir notaya denk gelecektir.

Velocity degeri, pratikte duyulmayacak notalardan maksimum nota seviyelerine kadar araligi kapsayan\
1'den 127'e kadar bir sayisal araliktadir. Temel olarak, muzik notasyonunda bulunan farkin olcegine\
denk gelir, asagidaki gorselde bu olcek degerleri gosterilmistir:

![*Velocity Values*](https://www.cs.cmu.edu/~music/cmsip/readings/MIDI%20tutorial%20for%20programmers_files/nuance-velocity-table.jpg)

Temel sentezleyicilerde, velocity degeri sadece, calinacak nota icin bir siddet degeri belirler, bu\
siddet notanin daha guclu veya yumusak bir ses efektine sahip olacagini belirtir.

Daha gelismis sentezleyicide, bu deger ayni zamanda sesi kalitesini de belirler. Aslinda, gercek\
pianoda bir notaya daha sert basmak sadece, notanin gurultulu cikmasini saglamaz, bunun yani sira\
*timber* adi verilen sesin kendi kalitesini de saglar. Bu olay, pratikte herhangi bir enstrumanda olan\
durumdur.

Velocity degerinin sifir oldugu ozel bir durum vardir. Velocity degerinin 0 oldugu **NOTE ON** mesaji\
**NOTE OFF** mesaji ile ayni anlama gelir, yani nota calmayi durdurur.

**NOTE OFF** mesaj yapisi su sekildedir:

 1. Status byte : 1000 CCCC
 2. Data byte 1 : 0PPP PPPP
 3. Data byte 2 : 0VVV VVVV
 
"CCCC" ve "PPP PPPP" degerleri **NOTE ON** mesajindaki ile ayni anlamdadir. "VVV VVVV" degeri hizi\
serbest birakma anlamina gelir ve nadiren kullanilir. Default olarak, 0 degerine ayarlanir.

## MIDI Dersleri 3 - Nota ve Akor Calma

Bir sentezleyiciye **NOTE ON** mesaji gonderdiginizde, bu nota calmaya baslar. Bu esnada, bir akor\
duymak icin farkli nota pitch degerleri iceren baska **NOTE ON** mesajlari gonderebilirsiniz. Ancak,\
calan notalarin kayitlarini tutmaya ihtiyaciniz olacaktir, bu sayede her notaya denk gelen **NOTE OFF**\
mesajlarini yollayabilirsiniz, aksi takdirde calan notalar sonsuza kadar durmadan calmaya devam\
edeceklerdir.

Bir ornege bakalim. Asagidaki notalari calmak icin gerekli olan MIDI mesajlari nelerdir?

![Example](https://www.cs.cmu.edu/~music/cmsip/readings/MIDI%20tutorial%20for%20programmers_files/Midi-example-1.jpg)

Muzigi duyabilmek icin zaman cizelgesine ihtiyac oldugu icin, kanal 1 uzerinden yukaridaki muzigi\
calabilmek icin sentezleyiciye gonderilecek MIDI mesajlarinin bir zaman siralamasi olmasi gerekir.

- t=0 : 0x90 - 0x40 - 0x40 (Start of E3 note, pitch = 64)
- t=0 : 0x90 - 0x43 - 0x40 (Start of G3 note, pitch= 67)
- t=1 : 0x80 - 0x43 - 0x00 (End of G3 note, pitch=67)
- t=1 : 0x90 - 0x45 - 0x40 (Start of A3 note, pitch=69)
- t=2 : 0x80 - 0x45 - 0x00 (End of A3 note, pitch=69)
- t=2 : 0x80 - 0x40 - 0x00 (End of E3 note, pitch=64)
- t=2 : 0x90 - 0x3C - 0x40 (Start of C3 note, pitch = 60)
- t=2 : 0x90 - 0x47 - 0x40 (Start of B3 note, pitch= 71)
- t=3 : 0x80 - 0x47 - 0x00 (End of B3 note, pitch= 71)
- t=3 : 0x90 - 0x48 - 0x40 (Start of C4 note, pitch= 72)
- t=4 : 0x80 - 0x48 - 0x00 (End of C4 note, pitch= 72)
- t=4 : 0x80 - 0x3C - 0x40 (End of C3 note, pitch = 60)

"t" saniye cinsinden zamani temsil eder. Dakikada 60 vurus ile calinir, bu nedenle her ceyrek nota 1\
saniye surer.

# MIDI Dersleri 4 - Enstruman Secimi
Su ana kadar, sentezleyicilerin, notalari calabilmek icin hangi sesi kullanmalari gerektigi konusunda\
hic bir sey soylenmedi. Sentezleyici, muhtemelen piyano veya kendisinin varsayilan enstrumanini kullanir.\

Daha onceden tanimlanmis 128 adet ses listesinden bir enstruman secmek icin bir MIDI mesaji vardir.\
Teoride her sentezleyici kendi enstruman listesine sahip olabilir, ancak General MIDI (GM) standarti\
uyumlulugu basitlestiren 128 adet enstruman listesi tanimlar. Cogu sentezleyici en azindan GM standarti\
ile uyumlu bir uyumluluk moduna sahiptir.

Enstruman degistirmek` icin kullanilan MIDI mesajina "program degisimi" mesaji adi verilir. Mesaj, bir\
**STATUS byte** bir de **DATA byte** alanindan olusur:

- Status byte: 1100 CCCC
- Data byte 1: 0XXX XXXX

CCCC 0'dan 15'e kadar MIDI kanallarini temsil eder, XXX_XXXX 0'dan 127'e kadar enstruman numarasini temsil\
eder. MIDI kanallarina benzer olarak, enstruman numaralari cogunlukla sentezleyicilerde ve GM listelerinde\
gorulur, numaralar benzer sekilde 1 ekleme ve cikarma yontemi ile cevrilir.

Ornegin, trampet enstrumaninin numarasi GM listesinde 57'dir, bu yuzden MIDI mesajinin XXX_XXXX kismi, hedef\
sentezleyici dogru enstruman sesini cikarmak icin 56 degerine ayarlanmalidir.

Mesajdan sonra sentezleyiciye gonderilen notalar trampet sesi ile calinir.

### Enstruman Ses Kalitesi

Fark ettiginiz uzere, hedef sentezleyiciye gonderilen MIDI mesajlari ses kalitesine dair herhangi bir\
bilgi icermemektedir. Trampet enstrumanini secen bir MIDI mesaji alindiktan sonra, ucuz, dusuk kalite bir\
ses karti sentezleyicisi oldukca kotu trampet sesi calarken, yuksek kalite bir ornekleyici guzel ve gercekci\
bir trampet sesi calacaktir.

## MIDI Dersleri 5 - Davul Enstrumanlari

Davul enstrumanlari ozel bir durum olustururlar, cunku bu enstrumanlar, piyano veya trampet gibi belli ses\
tonlarina sahip degillerdir. Ozel bir MIDI kanali, davul enstrumanlarinin playback'ini gondermek icin\
kullanilir. Genel MIDI'de, bu kanal 10 numarali kanaldir, ancak davul seslerini almak icin baska kanallar\
kullanan sentezleyiciler de bulabilirsiniz.

Davul enstrumanlarina kanal atamasi durumunda, (ayrica, genelde sentezleyicilerde ozel ses efektleri de dahil\
edilir) uygun tonlu **NOTE ON** ve **NOTE OFF** mesajlari hangi davul veya ses efektinin calinacagini secmek\
icin kullanilirlar.

Ornegin; 10 numarali kanalda bass davul enstrumani calmak icin asagidaki **NOTE ON** mesaji gonderilir.

- 0x99 0x23 0x40

0x99, 10 numarali kanali kullanan **NOTE ON** mesaji icin **STATUS byte** degeridir. 0x23 onluk tabanda 35\
degerine denk gelir, bu deger GM listesinde akustik bas davul icin kullanilan nota numarasidir. 0x40 onluk\
tabanda 64 sayisina denk gelir, bu deger mezzo forte farki civarinda bir hizi temsil eder. Sonrasinda,\
**NOTEO OFF** mesajininin da gonderilmek gerekir:

- 0x89 0x23 0x00

## MIDI Dersleri 6 - MIDI Kanallarinin Kullanilmasi

MIDI protokolu 16 farkli MIDI kanalina kadar kullanabilir. Her kanalin kendi durum bilgisi vardir, ornegin;\
kullanilacak enstruman tanimlanir, calinacak notalar belirlenir, ses ve panoramik gibi degerler ayarlanir vb\

Farkli MIDI kanallari kullanilarak, her bir kanal icin belli bir enstruman tanimlanabilir. MIDI kanallarina\
notalar gonderilerek, o kanaldaki enstruman sesi cikarilir.

Asagida uc enstrumanli bir ornek bulunmaktadir.

![Three Instruments](https://www.cs.cmu.edu/~music/cmsip/readings/MIDI%20tutorial%20for%20programmers_files/Midi-example-2.jpg)

Saksafon, piyano ve bas davul, sirasi ile 1, 2 ve 10 numarali kanallari kullanacaklardir. Buna uygun MIDI\
mesaj siralamasi asagida verilmektedir.

Ilk olarak 'program change' mesaji gonderilir, bu sayede hangi kanalda hangi enstrumanin calinacagi belirtilir:

- t=0 : **0xC0 0x41** (Alto Saksafon = 66 ==> 65 olarak kodlanir = 0x41)
- t=0 : **0xC1 0x00** (Piano = 1 ==> 0 olarak kodlanir)
- t=0 : **0xC9 0x00** (Standart Davul Kiti = 1 ==> 0 olarak kodlanir)

Genel MIDI listesindeki davullar icin, 'program change' mesajinin verisi 1'dir (0 olarak kodlanir). Bazi\
sentezleyiciler, Caz kitleri, Orkestral kitler, Elektro kitler gibi farkli davul seslerini de saglarlar.

Sonrasinda, daha onceden aciklanan notalari gonderilir. Burada dikkat edilmesi gereken nokta, hizi 0 olan\
**NOTE ON** mesajlari kullanilmasidir, bu mesaj **NOTE OFF** mesajina denk gelir. Bu mesaj pratikte siklikla\
kullanilir.

- t=0 : **0x90 0x48 0x40** (C4 Saksafonu baslat, pitch = 72)
- t=0 : **0x91 0x3C 0x40** (C3 piyanoyu baslat, pitch = 60)
- t=0 : **0x91 0x43 0x40** (G3 piyanoyu baslat, pitch = 67)
- t=0 : **0x91 0x4C 0x40** (E4 piyanoyu baslat, pitch = 76)
- t=0 : **0x99 0x23 0x40** (Bas davulu baslat, pitch = 35)
- t=1 : **0x90 0x48 0x00** (C4 saksafonu durdur)
- t=1 : **0x99 0x23 0x00** (Bas davulu durdur)
- t=1 : **0x90 0x4A 0x40** (D4 saksafonu baslat, pitch = 74)
- t=2 : **0x90 0x4A 0x00** (D4 saksafonu durdur)
- t=2 : **0x90 0x4C 0x40** (E4 saksafonu baslat, pitch = 76)
- t=2 : **0x99 0x23 0x40** (Bas davulu baslat)
- t=3 : **0x90 0x4C 0x00** (E4 saksafonu durdur)
- t=3 : **0x99 0x23 0x00** (Bas davulu durdur)
- t=3 : **0x90 0x4F 0x40** (G4 saksafonu baslat, ptich = 79)
- t=4 : **0x90 0x4F 0x00** (G4 saksafonu durdur)
- t=4 : **0x91 0x3C 0x00** (C3 piyanoyu durdur)
- t=4 : **0x91 0x43 0x00** (G3 piyanoyu durdur)
- t=4 : **0x91 0x4C 0x00** (E4 piyanoyu durdur)

## MIDI Dersleri 7 - MIDI Kontrolculeri

128 tane tanimlanmis MIDI kontrolcusu vardir, ancak pratikte onlardan sadece bir kaci kullanilir.\
MIDI kontrolculerinin amaci, notalari calan sentezleyicilerdeki ses, panoramik (stereoda sesin soldan\
saga dogru hareket etmesi gibi), eko gibi parametreleri ayarlamaktir.

Asagidaki gibi bir mesaj yapisi insa edilir:

- Status byte: 1011 CCCC
- Data byte 1: 0NNN NNNN
- Data byte 2: 0VVV VVVV

CCCC kanal numarasi, NNN_NNNN kontrolcusu numarasi ve VVV_VVVV kontrolcuye atanan degeri temsil eder.

En genel kontrolcu numaralari asagidaki gibidir:

- 0 = Ses bank secimi (MSB)
- 1 = Modulasyon tekerlegi, genelde vibrato veya tremolo efektlerine atanir
- 7 = Enstrumanin ses seviyesi
- 10 = Panoramik (0 = sol, 64 = orta, 127 = sag)
- 11 = Aciklama (sentezleyiciye bagli olarak, bazen ses kontrolu icin kullanilir)
- 32 = Ses bank secimi (LSB)
- 64 = Surdurme pedali (Sustain pedal) (0 = pedal yok; >= 64 => pedal acik)
- 121 = Butun kontrolculer kapali (bu kanal icin olan tum kontrolcu degerlerini temizler, varsayilana donerler)\
- 123 = Butun notalar kapali (guncel olarak calan butun notalari durdurur)

__Ses ve Hiz__\
Ornegin; kanal 1'de calan enstruman icin ses seviyesi 100'e ayarlanmak isteniyorsa, asagidaki mesaj gonderilir:

 - **0xB0 0x07 0x64**

Ses degisiminin, sonrasinda calacak notalar uzerinde de etkisi olur. Sentezleyici, ayarlanan ses seviyesini\
degistirilene kadar saklar.

Bir notanin hizi **NOTE ON** mesaji ile birlikte gonderilir. Nota calmaya basladigi anda hiz degeri degistirilemez,\
bu yuzden nota calmaya basladiktan sonra o notanin seviyesini degistirmek icin ses kontrolcusu kullanilabilir.

Bir kresendo (artarak hizlanan ses) yaratmak icin, artan ses degerlerinin gonderilmesi gerekmektedir.

Kullanilan ses ve hiz degerleri arasindaki dengenin daima saglanmasi gerekir ki boylece her ikisi de dogru\
deger araliginda olurlar. Eger onlardan birisi yavas kalirsa, notalar dogru bir sekilde duyulmayabilir, hatta\
diger deger maximum seviyede olabilir. Her iki deger efekti, notanin gercek gurultusunu belirlemek icin birbiri\
ile carpimsaldir.

__Enstruman Secimi__\
Daha onceden de gordugumuz gibi, bir sentezleyicide 'program change' mesajinin 128 degeri yardimi ile bir ses\
secilebilir. Ses bank secim mesajlari (LSB ve MSB), 'program change' mesajina ek olarak kullanilabilir.

Bir sentezleyici bir yada daha fazla ses banki icerebilir, her bir bank 128 ses icerir. Eger sentezleyicinin\
ozel bir ses banki kullanilmak istenirse, ilk olarak yeni bir bank aktive etmek ve akabinde 'program change'\
mesaji gondermek gerekir.

Ornek olarak, eger LSB=1, MSB=5 olan bir bankta, kanal 1'den 3 numarali ses kullanilmak istenirse, asagidaki\
gibi bir MIDI mesaj serisi gonderilmelidir:

- **0xB0 0x00 0x05** (MSB ses bank secimi)
- **0xB0 0x20 0x01** (LSB ses bank secimi)
- **0xC0 0x02** (guncel banktaki ses secimi)

Bu mesaj serisi alindiktan sonra, sentezleyici ozel bir ses ile asagidaki notalari calacaktir.

MSB ve LSB 0'dan 127'e kadar bir aralikta oldugu icin, toplamda secilebilecek 128 x 128 = 16384 olasi ses\
banki vardir. Pratikte, sadece bir kac tanesi uygulanir.

'program change' ve ses banklari uygunlugu hakkindaki bilgiler, kullanilan sentezleyicinin MIDI uygulama\
cizelgesinde ve MIDI spesifikasyonunda bulunabilir.

## MIDI Dersleri 8 -  Pitch Bend (Perde Bukme)

'Pitch Bend' mesaji, guncel MIDI kanalinda calan notanin perdesini degistirmek icin kullanilir. Asagida\
ornek bir mesaj gosterilmektedir:

- Status byte : 1110 CCCC
- Data byte 1 : 0LLL LLLL
- Data byte 2 : 0MMM MMMM

CCCC, MIDI kanalini temsil eder, LLL_LLLL perde bukme degerinin LSB'si, MMM_MMMM ise MSB'sidir. Perde\
bukmenin 14 bitlik degeri tanimlanir, bu sayede 0x2000 degeri notanin normal perdesine denk gelir.\
(pitch degisikligi olmaz). 0x2000 uzerindeki sayilari kullanmak (0x3FFFF'e kadar) pitch degerini artirir,\
bu degerin altinda bir deger (0x0000'a kadar) pitch degerinin azaltir. 'Pitch change' araligi simetriktir\
ve sentezleyici icerisinde kendi kendine siklikla ayarlanabilir. En yaygin aralik degeri, standart nota\
pitch degeri civarinda +/-2 yarim tondur.

'Pitch Bend' genelde glissando ve gitar bukme efektleri yaratmak icin kullanilir. Bunu yapabilmek icin\
'Pitch Bend' mesajinin surekli seri olarak gonderilmesi gerekmektedir. Gercek zamanli pitch degerini\
degistirebilmek icin, bu islem yeteri kadar siklikta yapilmalidir, boylelikle kulak curve (kivrim, egri)\
icerisinde fazla adimlari duymaz.

Ornegin; bir notayi yarim nota bukmek icin, 0x3000 degeri gonderilmelidir:

- **0xE0 0x00 0x60**

Aslinda, 0x3000 degeri, MSB ve LSB parcalari icin 0x60 ve 0x00 olan iki adet 7 bitlik degere bolunmelidir.

## MIDI Dersleri 9 - Notalarin Resetlenmesi (Resetting notes)

Gordugumuz uzere, butun **NOTE ON** mesajlarina karsilik gelen **NOTE OFF** mesajlari olmalidir, aksi takdirde\
notalar sonsuza kadar calmaya devam edecektir.

Calan notanin resetlenmek istenildigi durumlar olabilir. Bunu yapabilmek icin temel olarak 4 yol vardir. Bazi\
sentezleyiciler bu yollarin hepsini kabul etmezler, bu yuzden farkli olasiliklari sunmak ilgi cekici olur.

__1 - MIDI Kontrolcu 123 Kullanmak__\
Bir MIDI kanalindan MIDI Controller 123 mesaji gonderilirse, sentezleyici kanalda calan tum notalari durdurur.\
Butun MIDI kanallarini resetlemek icin, her kanala mesaj gonderilir. Dikkat edilmesi gereken nokta, butun\
sentezleyiciler bu mesajlara cevap vermezler.

__2 - MIDI Reset Mesaji__\
Bu mesaj, data kismi olmayan bir status mesajidir, **0xFF**. Mesaj, sentezleyiciyi power-on varsayilanina getirir,\
boylece calan butun notalar durur. Bu mesaji fazla kullanmamak gerekir, cunku mesaj sentezleyici tamamiyle\
resetler, sadece calan notalari durdurmaz.

__3 - MIDI NOTE OFF__\
Her bir kanal ve nota pitch degeri icin her kanala bir adet MIDI NOTE OFF mesaji gonderilir. Bu tam bir cozum\
olmasina ragmen kullanilan MIDI donanimina gore bazi reaksiyon zamanlarina sahip olan bir cok MIDI mesajinin\
gonderilmesi gerekir.

__4 - MIDI NOTE OFF - Optimized__\
Bu durumda, her kanal icin gonderilen **NOTE ON** ve **NOTE OFF** mesajlarinin kayitlarini tutmak icin bir tablo\
kullanilabilir. Her kanal icin 128 byte degerinde bir buffer kullanilabilir, bu buffer gonderilen **NOTE ON**\
mesajlarinin sayisini tutar ve her **NOTE ON** mesajinda artirilip her **NOTE OFF** mesajinda azaltilir. Akabinde,\
tum notalari resetlemek istedigimizde, basitce tabloya gidip calan notalar icin **NOTE OFF** mesajlari gonderilir.
