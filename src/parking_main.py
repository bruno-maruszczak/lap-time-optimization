import os
# local files
from parking.parking import ParkingLot, Car
from parking.OMPL import OMPL
from parking.plot import Plot

def main():
    parking_lot = ParkingLot()
    car = Car()

    plot_bounds = parking_lot.PLOT_BOUNDS
    parking_lot.init_parking_slots(car.CAR_LEN,car.CAR_WID)

    ompl = OMPL(plot_bounds, car, parking_lot)
    
    colours = {"bicycle": "deepskyblue", "4ws": "orange"}
    results = {m: ompl.create_planner(m) for m in ("bicycle", "4ws")}

    plot = Plot(parking_lot, results, plot_bounds, colours, car)
    plot.static_plot(os.path.join(os.getcwd(), "src/parking/parking_paths.png"))
    plot.animate(os.path.join(os.getcwd(), "src/parking/parking_animation.gif"))


if __name__ == "__main__":
    main()