Based on [this repository](https://github.com/joedavison17/dissertation/)
# lap-time-optimization

## Running the scripts
To install the required dependencies run:
```bash
python3 -m venv ./venv
source ./venv/bin/activate
pip install -r ./requirements.txt
```

Plotting depends on TeX so you may be required to install appropriate packages on your system

On NixOS you can use the development shell from this repository instead:
```bash
nix-shell # !!! Only use it on NixOS, this shell overrites $LD_LIBRARY_PATH which may break other systems
```

To run the optimisation run the `__main__.py` script in `src` directory. You can run the following to get information about script usage:
```bash
python src/__main__.py --help
```
Example usage:
```
python3 src/__main__.py --nonlinear --plot-all ./data/tracks/buckmore.json ./data/vehicles/tbr18.json
```
```
python3 src/__main__.py --bayes --plot-all ./data/tracks/buckmore.json ./data/vehicles/tbr18.json
```

*\[PL\]*
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
### Non-linear
![nonlinear](./data/plots/buckmore/nonlinear/trajectory.png)

