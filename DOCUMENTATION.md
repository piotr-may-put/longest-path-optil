Oto propozycja dokumentacji w formacie Markdown (`.md`), która opisuje dostarczony kod realizujący heurystyczne poszukiwanie ścieżki o maksymalnej wadze (problem NP-trudny, pokrewny do problemu najdłuższej ścieżki/komiwojażera).

Możesz skopiować poniższą treść i zapisać ją jako np. `dokumentacja.md`.

---

# Dokumentacja algorytmu: Poszukiwanie ścieżki o maksymalnej wadze w grafie

Niniejszy dokument opisuje działanie, wykorzystane struktury danych oraz propozycje udoskonaleń dla heurystycznego algorytmu wyznaczania najcięższej ścieżki (ang. *longest path problem* / *maximum weight path*) w grafie nieskierowanym. Algorytm działa w z góry określonym oknie czasowym (ang. *time limit*).

## 1. Krótki opis działania

Algorytm opiera się na strategii **Iterated Local Search (ILS)** z elementami **randomizacji** oraz **rotacji Pósy** (Pósa rotations). Składa się z dwóch głównych faz.

### Pseudokod (High-level)

```text
1. Wczytaj graf i usuń krawędzie wielokrotne (zostaw najcięższe).
2. [Faza 1: Inicjalizacja] Wykonaj 8 prób (ograniczonych czasowo):
   a. Wygeneruj losowe drzewo rozpinające za pomocą DFS (krawędzie mają zmodyfikowane wagi na podstawie stopnia wierzchołka).
   b. Znajdź najcięższą ścieżkę w tym drzewie (średnicę) używając programowania dynamicznego.
   c. Próbuj przedłużyć ścieżkę na obu jej końcach (zachłannie).
   d. Wykonaj przeszukiwanie lokalne (poprawa ścieżki).
   e. Zapisz najlepszą znalezioną ścieżkę.
3. [Faza 2: Optymalizacja / Random Kick] Powtarzaj aż minie TIME_LIMIT (25s):
   a. Z prawdopodobieństwem 50% odwróć całą ścieżkę (zamiana początku z końcem).
   b. Z prawdopodobieństwem 50% wykonaj rotację Pósy (złam ścieżkę i zrób nowy koniec).
   c. Destrukcja (Kick): Usuń losowej długości końcówkę ścieżki (najczęściej krótką, rzadziej dłuższą).
   d. Odbudowa: Zachłannie przedłużaj nowo powstały koniec ścieżki.
   e. Przeszukiwanie lokalne: Uruchom algorytm poprawy (wymiana wierzchołków) dla zmodyfikowanego fragmentu ścieżki.
   f. Jeśli nowa ścieżka jest lepsza (cięższa), zapisz ją. W przeciwnym razie przywróć poprzednią (najlepszą) ścieżkę na odrzuconym fragmencie.
4. Wypisz wynik (rozmiar ścieżki oraz jej wierzchołki).

```

## 2. Algorytmy i struktury danych

### Struktury danych

1. **Złożona lista sąsiedztwa (`adj`):** Reprezentuje graf. Przechowuje pary `(sąsiad, waga)`. Listy są posortowane w celu szybkiego wyszukiwania, a duplikaty krawędzi są usuwane (zostaje krawędź o maksymalnej wadze).
2. **Tablica pozycji (`pos`):** Kluczowa struktura dla wydajności algorytmu. Indeksowana numerem wierzchołka, przechowuje jego aktualną pozycję w ścieżce (lub `-1`, jeśli do niej nie należy). Pozwala w czasie $O(1)$ sprawdzać, czy wierzchołek jest już wykorzystany.
3. **Tablica mapowania krawędzi (`edgeToNext`):** Używana w fazie lokalnego ulepszania. Działa jak szybka mapa (hash map) w czasie $O(1)$ dla sąsiadów konkretnego wierzchołka, co znacząco przyspiesza obliczanie nowych wag krawędzi przy podmienianiu wierzchołków.
4. **Wektor reprezentujący ścieżkę (`path`):** Przechowuje aktualnie analizowany ciąg wierzchołków.

### Użyte algorytmy i heurystyki

1. **Losowy DFS + DP (Dynamic Programming):** Algorytm na początku buduje losowy las DFS, modyfikując wagi krawędzi o tzw. karę za stopień (wierzchołki o wysokim stopniu "kosztują" więcej, by zachować je na później). Następnie za pomocą programowania dynamicznego (DP na drzewach) znajduje w tym lesie najcięższą ścieżkę.
2. **Rotacje Pósy (Pósa Rotations):** Technika ucieczki ze "ślepych zaułków". Jeśli koniec ścieżki nie ma wolnych sąsiadów, znajdujemy krawędź łączącą koniec ścieżki z jej środkiem. Ścieżka tworzy cykl z "ogonem". Odwracamy odpowiedni fragment cyklu, tworząc nową ścieżkę o tej samej długości, ale z nowym wierzchołkiem końcowym, co często pozwala na dalszą ekspansję.
3. **Przeszukiwanie lokalne (Local Search / `improve`):**
* **Wymiana wierzchołków ($a \to c \to b$ na $a \to x \to b$):** Sprawdza, czy można wyrzucić jeden wierzchołek ze środka ścieżki i wstawić inny, nieużywany, jeśli suma wag wzrośnie.
* **Wstawianie wierzchołków:** Szuka możliwości wsunięcia jednego lub dwóch nowych (nieużywanych) wierzchołków pomiędzy dwa sąsiadujące na ścieżce ($a \to x \to b$ lub $a \to x \to y \to b$).



## 3. Elementy do udoskonalenia

Poniżej znajdują się propozycje optymalizacji, które mogą podnieść punktację algorytmu (jakość rozwiązania) lub liczbę sprawdzanych kombinacji w wyznaczonym czasie, bez psucia jego podstawowej logiki.

### A. Zrównoleglenie (Wielowątkowość)

* **Koncepcja:** Algorytm obecnie korzysta z jednego wątku. Pętla z fazy 2 jest w pełni niezależna od innych równoległych prób (wymagałaby jedynie lokalnych kopii struktur danych). Można uruchomić przeszukiwanie na wielu wątkach i na końcu wybrać globalne maksimum.
* **Oczekiwany efekt:** Prawie liniowy przyrost liczby iteracji algorytmu w zależności od liczby rdzeni (np. $4\times$ więcej iteracji na 4 rdzeniach). Skutkuje to dużo większą szansą na wyjście z lokalnych maksimów.
* **Koszt wdrożenia:** Niski/Średni (~1-2 godziny pracy). Użycie `#pragma omp parallel` (OpenMP) lub `std::thread`, plus zhermetyzowanie zmiennych globalnych (jak `path`, `pos`, `curWeight`) wewnątrz klasy/struktury.

### B. Akceptowanie gorszych rozwiązań (Simulated Annealing)

* **Koncepcja:** W fazie 2, jeśli odrzucona końcówka i nowo wygenerowana ścieżka dają gorszy wynik (`curWeight < bestWeight`), kod sztywno cofa się do najlepszego rozwiązania (`setPath`). Warto dodać prawdopodobieństwo akceptacji gorszego rozwiązania, malejące wraz z upływem czasu.
* **Oczekiwany efekt:** Algorytm zyska potężne narzędzie do ucieczki z ekstremów lokalnych. Pozwoli to na eksplorację zupełnie nowych części przestrzeni rozwiązań. Wynikowa waga ścieżki powinna zauważalnie wzrosnąć w grafach o złożonej topologii.
* **Koszt wdrożenia:** Niski (~1 godzina). Wymaga modyfikacji pętli głównej i dodania wyliczeń funkcji wykładniczej na bazie czasu (`elapsed() / TIME_LIMIT`).

### C. Pełna 2-optymalizacja (2-opt dla wag)

* **Koncepcja:** Funkcja `improve` analizuje wstawianie i podmienianie pojedynczych wierzchołków. Brak w niej klasycznego ruchu `2-opt` (usunięcie dwóch nieprzyległych krawędzi i zamienienie ich miejscami przez odwrócenie podścieżki), popularnego w problemach typu TSP.
* **Oczekiwany efekt:** Wykorzystywanie głębszych zależności. Długie fragmenty ścieżki ułożą się bardziej optymalnie.
* **Koszt wdrożenia:** Średni/Wysoki (~3-4 godziny). Należy zaimplementować to z rozwagą, aby nie stworzyć wąskiego gardła wydajności (złożoność rzędu $O(L^2)$, gdzie $L$ to długość ścieżki).

### D. Strojenie hiperparametrów (Tuning)

* **Koncepcja:** Kod operuje na sztywnych stałych (magiczne liczby: np. prawdopodobieństwa w `rng() % 20`, rzuty typu `rng() % 5000`, parametry kar `penalties = {0, 5, 15, 30}`).
* **Oczekiwany efekt:** Średnio 2-5% lepsze wyniki na zestawach testowych, jeśli stałe zostaną dobrane przy użyciu algorytmu uczenia maszynowego (np. optymalizacja bayesowska) do konkretnej charakterystyki wejściowych grafów.
* **Koszt wdrożenia:** Wysoki (tylko pod kątem czasu CPU/testów - wymaga napisania skryptu testującego, np. w Pythonie). Samo zastąpienie magicznych liczb stałymi i wystawienie ich jako argumenty programu zajmie kilka minut.