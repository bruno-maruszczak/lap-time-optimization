Based on [this repository](https://github.com/joedavison17/dissertation/)
# lap-time-optimization

## example of running program
python3 src/__main__.py --nonlinear --plot-all ./data/tracks/buckmore.json ./data/vehicles/tbr18.json

python3 src/__main__.py --bayes --plot-all data/tracks/buckmore.json data/vehicles/tbr18.json


## Założenia

Założeniem projektu było wykonanie algorytmów optymalizujących prędkość przejazdu modelu po torze i porównanie ich wyników, t.j.
prędkości maksymalnej, prędkości średniej, długości znalezionej ścieżki oraz czasu przejazdu. Wykorzystano kod stworzony przez 
Joe Davison'a, jako bazę. Dzięki temu zostały już zaimplementowane metody obliczania czasu, plotowania wyników etc.  

## Realizacja

Niestety przez brak czasu i napotkane problemy podczas implementacji (które znacznie utrudniły wykonanie projektu)
udało się zaimplementować tylko dwa algorytmy.
1. Pierwszy jest oparty na dokumentacji https://arxiv.org/pdf/2002.04794. 
Jest to Bayesian optimisation. 
Metoda ta w dużym skrócie oparta jest na wylosowaniu trajektorii w postaci jednowymiarowej listy reprezentującej pozycję na torze, 
policzeniu dla każdej z nich czasu przejazdu, stworzeniu z nich bazy danych i
wyuczeniu regrosora Gaussowskiego. Tor reprezentowany jest zgodnie z */documents/dissertation.pdf*. 
Wypróbowano jeszcze dwie reprezantacje w ukladzie zmiennych toru, jednak nie dawały jednoznacznych rezultatów. 

2. Drugi to non-linear optimisation, dokonuje wylosowania ścieżek na trasie, wybiera 10 najlepszych i dokonuje na nich optymalizaci nieliniowej, 
za pomocą algorytmu *COBYLA*. 

## Wyniki dla toru buckmore
Wyniki przedstawiono w tabeli: 
|Method:       |   curvature   | Compromise | Lap Time | Bayesian Optimisation |  non-linear optimisation |
| ----------- |:-------------:|:----------:|:----------:|:-------------:|:----------------------:|
|Lap time      |     39.934    |   37.810  |   40.892    |     36.227    |           36.178      |
|Run time      |     2.037     |   35.233  |   47.472    |     22.396    |           106.063     |
|Path Length   |     860.772   |   790.462 |   830.327   |     773.561   |           772.140     |
|Max velocity  |     40.050    |   40.833  |   37.790    |     41.365    |           43.333      |
|Mean velocity |     23.414    |   22.958  |   22.293    |     23.908    |           23.833      |


Oraz na plotach:
### Curvature
![curvature](./data/plots/buckmore/curvature/trajectory.png)
### Compromise
![compromise](./data/plots/buckmore/compromise/trajectory.png)
### Lap Time
![laptime](./data/plots/buckmore/laptime/trajectory.png)
### Bayesian
![bayesian](./data/plots/buckmore/bayesian/trajectory.png)
<!-- ![curvature](./data/plots/buckmore/nonlinear/trajectory.png) -->



Lap Time ma najdłuższy czas, ponieważ wyłącza się po paru iteracjach. 

*for 10 iterations and single core








## problemy:
problemów może być parę. 
- po pierwsze  dodatnie wolosowane wartości dla w_t nie są zawsze po jednej stronie toru. To może wprawadzać błędy w postaci nagłych skoków z jedne części na drugą. Jk zapewnić żeby byly po jednej stronie?
- po drugie nie wiem czy jest to minimalizowane tak jak oni chcą w dokumentacji. Chat napisał funkcję, która minimalizuje teoretycznie, ale czy faktycznie robi to dobrze?
- po trzecie trzeba zobaczyć czy podanie jako warunków początkowych do minimalizacji waypointów z najlepszym czasem nie poprawi wyników. -> done, nie wiem czy poprawiło XD



  TODO: komentarze dopisac