Based on [this repository](https://github.com/joedavison17/dissertation/)
# lap-time-optimization

# example of running program
python3 src/__main__.py --curvature --plot-all ./data/tracks/buckmore.json ./data/vehicles/tbr18.json

python3 src/__main__.py --bayes --plot-all data/tracks/buckmore.json data/vehicles/tbr18.json


## problemy:
problemów może być parę. 
- po pierwsze  dodatnie wolosowane wartości dla w_t nie są zawsze po jednej stronie toru. To może wprawadzać błędy w postaci nagłych skoków z jedne części na drugą. Jk zapewnić żeby byly po jednej stronie?
- po drugie nie wiem czy jest to minimalizowane tak jak oni chcą w dokumentacji. Chat napisał funkcję, która minimalizuje teoretycznie, ale czy faktycznie robi to dobrze?
- po trzecie trzeba zobaczyć czy podanie jako warunków początkowych do minimalizacji waypointów z najlepszym czasem nie poprawi wyników. -> done, nie wiem czy poprawiło XD



 ConvergenceWarning: lbfgs failed to converge (status=2):
ABNORMAL_TERMINATION_IN_LNSRCH.

Increase the number of iterations (max_iter) or scale the data as shown in:
    https://scikit-learn.org/stable/modules/preprocessing.html
  _check_optimize_result("lbfgs", opt_res)

  TODO: komentarze dopisac