## Przykład

```
# Budowanie plików binarnych
./build_osrm.sh
cd gen-graph
make
cd ../oracles
make
cd ..

# Ściągnięcie i przetworzenie mapy
wget http://download.bbbike.org/osm/bbbike/Gliwice/Gliwice.osm.gz
gunzip Gliwice.osm.gz
./oracles/src/osrm/build/osrm-extract -p oracles/src/osrm/profiles/profile.lua Gliwice.osm
./oracles/src/osrm/build/osrm-contract Gliwice.osrm

# Generowanie grafu, testu oraz przetestowanie wyroczni
./gen-graph/gen-graph Gliwice.osm > gliwice_graph
./oracles/gen_test gliwice_graph > gliwice_test
./oracles/run_tests Gliwice.osrm gliwice_graph gliwice_test
```

## Struktura repozytorium

* gen-graph: Tu znajduje się parser map / generator grafów. 
* oracles: Tu znajdują się wyrocznie oraz generator testów. W oracles/src/osrm jest kod osrm-backend.
* build_osrm.sh: Krótki skrypt budujący biblioteki oraz programy osrm.


## Dłuższy opis

Programy korzystają z kilku bibliotek (pierwsza lub dwie pierwsze z nich mogą wymagać doinstalowania):
* GeographicLib: Z tego korzysta gen-graph aby dokładnie wyliczać odległości z koordynat. Jeśli nie jest ona dostępna w
managerze pakietów można ją pobrać [tutaj](https://geographiclib.sourceforge.io/).
* mpfr: Powinna być dostępna w managerze pakietów jeśli nie jest dostępna [tutaj](http://www.mpfr.org/).
* boost - system filesystem iostreams program_options regex date_time thread
* tbb

Następnie należy zbudować osrm np za pomocą skryptu *build_osrm.sh*.
Po czym przechodzimy do obu z folderów i wykonujemy polecenie *make*

Powinno to zbudować pliki wykonywalne:
* gen-graph:
  * gen-graph: Generuje graf z plików osm. Mapy można pobrać [tutaj](https://www.openstreetmap.org/export)
lub można też znaleźć gotowe mapy miast [tutaj](http://download.bbbike.org/osm/bbbike/). Mapy są dostępne w paru
formatach generator obsługuje jedynie format osm. Program przyjmuje jeden argument: plik osm i na standardowe
wyjście wypisuje wygenerowany graf. 
* oracles
  * gen_test: Generator testów z wygenerowanych grafów. Przyjmuje jeden argument: wygenerowany graf i na standardowe wyjście
wypisuje test
  * run_test: Testuje wyrocznie na podanym teście. Przyjmuje trzy argumenty: plik .osrm
(jak go otrzymać z mapy osm jest opisane poniżej),  wygenerowany graf, wygenerowany test. Na standardowe wyjście wypisuje
wyniki. Na razie generator testów i testerka są dość ubogie.
  * oracle_main: Oryginalna testerka

### Jak otrzymać plik osm -> osrm

Testerka testuje również wyrocznie opartą na osrm co wymaga przetworzenia mapy również ich programami. Po wykonaniu skryptu
build_osrm.sh powinien utworzyć się folder *oracles/src/osrm/build*  w którym znajdują się biblioteki oraz pliki
wykonywalne projektu osrm. Co można przeczytać w komentarzu *oracles/src/osrm_oracle.cpp:26* w zależności od
użytego algorytmu mapa musi zostać odpowiednio przetworzona.

Przetwarzanie mapy zawsze zaczyna się od *osrm-extract* któremu oprócz ścieżki do mapy musimy również podać profil za pomocą
parametru -p. Przygotowałem profil który mam nadzieje jest odpowiedni w *oracles/src/osrm/profiles/profile.lua*. Wykonanie tego programu
generuje pliki osrm (w tym samym miejscu co oryginalna mapa) który następnie podajemy jako argument do kolejnych
programów w zależności od użytego algorytmu.

Uwaga w wyniku przetworzenia mapy powstaje wiele plików i pomimo iż do testerki podaje się tylko plik .osrm pozostałe
pliki również są potrzebne i biblioteka osrm zakłada że znajdują się one w tym samym miejscu co plik .osrm.
