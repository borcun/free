==C++ Threading Kullanmak Icin 5 Neden==
Gelecek 10 15 yilda, her iki yilda bir islemci hizini iki katina cikacagini soyleyen Moore Kanunu'nun
sonuna gelebiliriz. Teoride, bu akimi yakalayabilmek icin transistorler cok kucuk parcalara ayriliyorlar,
hatta suanda bile ureticiler ve gelistiriciler, bir talimat ile birden cok veri islemine olanak saglayan
farkli paralellestirme ( vectorization gibi ) formlari kullanma ve islemciler uzerine birden fazla
cekirdek ekleme konusunda ilerleme uzerine calisiyorlar.

Birden fazla islemcinin sadece tek bir cekirdege sahip oldugunda, C++ gibi eski diller ile, daha fazla 
cekirdegi kullanan kod yazmak daha zordur. Bununla birlikte, farkli metotlar dusunuldu. Bu makalede,
ayni kodla iki farkli teknige bakacagiz.

==5 Neden?==
Asagida daha fazla cekirdek kullanan kod yazmak icin anahtar sebepler var.

- Islem suresini hizlandirmak
- Islemler arasi etkilesim olusturmak
- Verileri ve I/O islemlerini ortusturmek
- Daha az kaynak kullanmak
- Paylasimi kolaylastirmak

Eger islemler birden fazla ayri hesaplamalara ayrilabilir ve birden fazla cekirdek ile bir CPU uzerinde
calisabilir, indigenmis calisma zamanina gecebilirlerse, islem suresi hizlandirabilir. Uygulama GUI
tabanli ise ve yogun hesaplama ozellikleri iceriyorsa, bir arka plan thread'i icerisinde gereksinimlerin
yuku cok fazla yer kaplamasa bile, GUI donmalari olusur. Ayni olay disk veya network tabanli buyuk I/O
islemleri icin de gecerlidir. Islemi birden fazla cekirdege dagitmak, bir thread icerisinde yapilan 
islemleri mukayese ettigimizde, kaynak kullaniminib azaldigi gozlemlenebilir.

Kendi kendinize bir multithread kod yaziyorsaniz, thread yaratma ve yonetme islemleri zor olabilir.
Zamanlamadan dolayi debug surecinde gozlemlenemeyen fakat urun icerisinde ortaya cikan bir suru problem
olacaktir.

==Thread Olmadan Threading==
Sonuc olarak hem thread kullanip hem de olasi zafiyetlerden korunmak istiyoruz. Bir kac kutuphane bize
bu fonksiyonu sunmaktadir: en populerleri Intel TBB ve acik kaynak OpenMP. Her iki kutuphane de C++ ve
Fortran dillerini desteklemektedir, hatta OpenMP C dili ile de kullanilabilir.

==Intel TBB==
Veri yapilari ve algoritmalar iceren TBB, is yukunu birden fazla cekirdekte dengelemek icin template 
yapilarini kullanir. Bir cekirdek gorevini bitirip serbest olabilir kaldiktan sonra, TBB dinamik olarak 
is yukunu cekirdeklere paylastirirken bu anda yeni bir duzenleme yapabilir. OpenMP'nin aksine TBB, herhangi
bir C++ derleyicisi ile calisir ve ozel bir destege ihtiyac duymaz. Sadece Intel TBB paylasimli kutuphaneleri
koda baglanmalidir, ardindan TBB namespace'i ile tum yapilara erisilebilir.

TBB'nin paralel calisan for, do, while, sort ve baska bir takim yapilari vardir. Ayrica, es zamanli queue,
vector ve hashmap gibi veri yapilarina ve calisma sureci boyunca baska bir process tarafindan mudahele edilmemesi
garantilenen, dusuk seviyeli memory allocation ve atomic operasyonlara da sahiptir.

Asagidaki ornekte, getValue fonksiyon cagrisi ile kullanilan buyuk verilerin parcalanmasi icin TBB kutuphanesine
ait parallel_for kullanilarak islem yuku azaltilip performans artiriliyor.

// Hold our outputs
std::vector outputs{ kMaxValues };
 
// Uses parallel for loop with a C++11 lambda to perform mapping into outputs
tbb::parallel_for(size_t(0), kMaxValues, [ &outputs ]( size_t i ) { outputs[i] = getValue(i); } );
 
// Do reduction
double output=0.0;
for(double v: outputs) { output += v;}

==OpenMP==
OpenMP, kodun parallellige ihtiyaci oldugu anda, kole threadleri calistiran bir lider thread'e sahip olarak calisir.
Intel TBB kutuphanesinin calismasinin aksine, derleyicinin OpenMP direktiflerinin harici olarak implement etmeye ihtiyaci
vardir. Microsoft Visual Studio C++ 2013 icin en guncel OpenMP, 4.0 versiyonudur. Ayni zamanda 2.0 versiyonunu da 
destekler.

OpenMP'i kod icerisinde kullanabilmek icin, derleyici icerisindeki kutuphaneyi aktif duruma getirmek gerekir. Bunun icin
paralellestirme yapilmasi istenilen kisimlara pragmalar eklenir. Ek olarak omp.h header dosyasini ekleyerek belli fonksiyonlari
cagirmak gerekir. Bunlar olasi thread sayisini belirleyen omp_set_num_threads() fonksiyonu ve calisma zamaninda dinamik thread
sayisina izin veren omp_set_dynamic() fonksiyonudur.

#define THREADS 8
 
#ifdef _OPENMP
   omp_set_num_threads(THREADS);
   omp_set_dynamic(THREADS);
#endif

Asagidaki kodda, 'pragma omp parallel for' ifadesi dongunun paralel calismasi icin kullanilmistir.

vector<node>* routes[num_cities - 1];
   
#pragma omp parallel for
  for (int i = 0; i < num_cities - 1; ++i) {
    vector<node>* search_nodes = new vector<node>();
    routes[i] = search_nodes;

    int dist = find_route(map, city_locs, i, *search_nodes);
    cost += dist;
 }

Critical section alanlari icin #pragma omp critical pragmasi kullanilabilir. OpenMP, paralel ve
paralel olmayan parcalar arasinda veri paylasiminda oldukca iyidir. Bir verinin, bir thread icin veya
paylasmak icin gizli olmasi veya olmamasi saglanabilir. Bu ozellik main thread icerisinde degiskeninin 
farkli bir sekilde ilklendirilmesi ile yapilir.

==Sonuc==
Sonuc olarak, C++ kodunuzun performasini threading ile artirmak ve olabilecek bir takim karisikliklardan 
ve risklerden uzak durmak istiyorsaniz Intel TBB veya OpenMP'den herhangi birini deneyebilirsiniz. Her iki
kutuphanede ucretsizdir.