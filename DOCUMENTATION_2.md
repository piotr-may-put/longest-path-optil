# Longest path – opis rozwiązania (`solution.cpp`)

Problem jest NP-trudny, więc używamy heurystyki: budujemy ścieżkę zachłannie, a potem poprawiamy ją lokalnie do końca limitu czasu (`TIME_LIMIT` = 29 s).

## Struktury

- `adj[v]` – lista sąsiadów `(sąsiad, waga)`, posortowana po sąsiedzie (wielokrotne krawędzie → zostaje najcięższa). `getWeight(a,b)` szuka wagi binarnie (0 = brak krawędzi).
- `path` – aktualna ścieżka, `pos[v]` – indeks `v` w `path` albo `-1`, jeśli `v` jest wolny.
- `curWeight` – waga aktualnej ścieżki, aktualizowana przy każdej operacji (`pushVertex`, `popVertex`, `reverseSubpath`, `improve`), więc nie liczymy jej od nowa.
- `minDirtyIdx` – najmniejszy indeks ścieżki zmieniony w bieżącym kroku; dzięki temu kopiujemy/przywracamy tylko zmieniony sufiks.
- `edgeToNext[x]` – tablica pomocnicza w `improve`: waga krawędzi `x–b` dla sąsiadów `b` (zamiast szukania binarnego).

## Operacje

1. **grow** – dopóki koniec ścieżki ma wolnego sąsiada, dokładamy najlepszego wg oceny
   `waga - degPenalty * (liczba wolnych sąsiadów) + szum`. Duże `degPenalty` = reguła Warnsdorffa (najpierw węzły, które łatwo „zgubić”).
2. **rotateEnd (rotacja Pósy)** – gdy koniec `v = path[k]` utknął, ale ma sąsiada `path[i]` na ścieżce:
   ```
   p0 .. pi  p(i+1) .. pk      (krawędź pk–pi istnieje)
   p0 .. pi  pk .. p(i+1)      (odwracamy fragment i+1..k)
   ```
   Ścieżka ma te same węzły, ale nowy koniec `p(i+1)`, z którego często da się dalej rosnąć. To zwykłe `reverse` na wektorze (`reverseSubpath`).
   `growEnd` łączy `grow` i rotacje: kończy po `maxTries` rotacjach bez nowego rekordu długości.
3. **improve** – lokalne poprawki od indeksu `from`:
   - zamiana `a-c-b` na `a-x-b` (x wolny), jeśli waga rośnie,
   - wstawienie `a-x-b` lub `a-x-y-b` w miejsce krawędzi `a-b`, jeśli waga rośnie.
4. **dfsPath – drzewo DFS + średnica** (rozwiązanie startowe):
   - puszczamy DFS (iteracyjnie, na stosie) z losowych korzeni, aż odwiedzi wszystkie węzły; sąsiadów odwiedzamy w kolejności „ciężka krawędź / mały stopień najpierw” (`sortedAdj`),
   - krawędzie `rodzic–dziecko` tworzą las DFS; każda ścieżka w tym drzewie jest też ścieżką w grafie,
   - szukamy najdłuższej (ważonej) ścieżki w drzewie: idąc od liści do korzenia, dla każdego węzła pamiętamy dwie najdłuższe gałęzie w dół (`maxDown1`, `maxDown2`) i dzieci, przez które idą (`bestChild1`, `bestChild2`); najlepszy węzeł `bestLca` maksymalizuje `maxDown1 + maxDown2`,
   - ścieżka = gałąź `bestChild2` (odwrócona) + `bestLca` + gałąź `bestChild1`.
     Dla grafu, który jest drzewem, to od razu optimum. Pomaga też, gdy graf ma wiele składowych (DFS obejmuje wszystkie).

5. **reroutePass – przebudowa okien** (cała ścieżka, nie tylko końce):
   - idziemy po ścieżce oknami: `a = path[i]`, wycinamy `path[i+1 .. j-1]` (1..`MAX_WINDOW` wierzchołków), wycięte stają się wolne,
   - DFS z `a` po wolnych wierzchołkach (głębokość ≤ wycięte + `EXTRA_DEPTH`, budżet `DFS_BUDGET` wywołań) szuka trasy, która wraca na ścieżkę w dowolnym `path[q]`, `q ∈ [j, i + MAX_SKIP]`, albo (przy końcu ścieżki) staje się nowym końcem,
   - zysk = waga trasy − waga starego fragmentu `a .. path[q]` (liczona z sum prefiksowych `prefix`); jeśli > 0, podmieniamy (pominięte wierzchołki są zwalniane),
   - przycinanie: jeśli nawet same krawędzie o wadze 100 nie pobiją rekordu, nie schodzimy głębiej,
   - nowa ścieżka jest składana raz na koniec przebiegu (jak w `improve`).
   Dzięki temu ścieżka „wchłania” wolne kieszenie w środku – kluczowe dla rzadkich grafów o strukturze siatki.
6. **twoOptPass – 2-opt**: `a→b … c→d` zamieniamy na `a→c … b→d` (odwrócenie fragmentu `b..c`), jeśli waga rośnie; kandydatów `c` bierzemy z listy sąsiadów `a`.

## Schemat

1. **Faza 1** – kilka startów z `dfsPath` (z różnymi karami `penalties`): z tej ścieżki rośniemy w obie strony (`extendBothEnds`), potem `improve`. Bierzemy najlepszą ścieżkę.
2. **Faza 2** – na zmianę: jeden przebieg `reroutePass` + `twoOptPass`, potem seria kopnięć. Długość serii kopnięć = czas przebiegu × (zysk/s kopnięć ÷ zysk/s przebiegów), ograniczone do [0.25, 20] – czas dostaje ta metoda, która ostatnio lepiej działa.
   Pojedyncze kopnięcie („kopnięcie” + hill climbing):
   - losowo odwracamy ścieżkę (pracujemy na drugim końcu) i czasem robimy losową rotację,
   - obcinamy koniec (zwykle do 100 węzłów, czasem do 5000) do miejsca `keepIdx`, które ma wolnego sąsiada,
   - odrastamy (`growEnd`), poprawiamy `improve` na zmienionym ogonie (co 100 iteracji na całej ścieżce),
   - jeśli waga ≥ najlepszej – przyjmujemy, w przeciwnym razie przywracamy poprzednią ścieżkę (od `syncFrom`).
