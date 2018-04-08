Environmental Variable olarak isimlendirilen sistem degiskenleri guncel profil icin daha onceden tanimlanmis olan degiskenlerdir.
printenv komutu ile guncel profildeki cevresel degiskenler gorulebilir.

Iki tip cevresel degisken vardir: local ve global. Local cevresel degiskenler sadece guncel profilde kullanilirlar. Ornegin terminalde
'MY_LOCAL_VAR=4' tanimlamasi yapildiktan sonra acilan baska bir terminalde MY_LOCAL_VAR degiskeni gorulmez. Degiskeni tanimlama ve
ilklendirme isleminden sonra global yapmak icin 'export MY_LOCAL_VAR' komutunun calistirilmasi gerekmektedir. Bu sayede MY_LOCAL_VAR
degiskeni tum profillerden gorulebilir.

set edilen cevresel degiskenler unset komutu ile kaldirilabilirler. Global degiskenler unset edilemezler.

PATH cevresel degiskeni sistemin calistirilabilir dosya dizinlerini ( /bin, /sbin, /usr/bin ... ) icerir.
LD_LIBRARY_PATH cevresel degiskeni sistemin kutuphane dizinlerini ( /lib, /lib32, /usr/lib ... ) icerir. ( Bkz: ldconfig )

home dizinindeki .profile, .bash_login veya .bash_profile dosyalari guncel profil icin tanimlanmis ilklendirme dosyalaridir. Bir
terminal acildigi zaman bu dosya tetiklenir. Kullanici, local olarak tanimlanmasini istedigi tanimlamalari bu dosyalarin icerisine
yazabilir. etc dizinindeki profile dosyasi sistem acilisinda bir kere calistirilir. Kullanici, global olarak tanimlamak istedigi
degiskenleri bu dosya icerisine yazabilir. Ayrica /etc/profile.d/ icerisinde bulunan calistirilabilir script dosyalari /etc/profile
tarafindan sistem acilisinda calistirilirlar. Kullanici uzun islemler icerebilecek startup scriptlerini bu dizine kopyalayabilir.