import os
# local files
from parking.parking import ParkingLot, Car, ParkingLotBitMap
from parking.OMPL import OMPL
from parking.plot import Plot

def main():
    # parking_lot = ParkingLot()
    parking_lot_bitmap = ParkingLotBitMap(os.path.join(os.getcwd(), "data/tracks/parking_layout.npy"))
    parking_lot_bitmap.get_obstacles()
    # print(parking_lot_bitmap.obstacles)

    plot_bounds = parking_lot_bitmap.PLOT_BOUNDS
    # Debug plotting
    # parking_lot_bitmap.plot()
    # parking_lot_bitmap.map_plot(plot_bounds)

    car = Car()

    # parking_lot.init_parking_slots(car.CAR_LEN,car.CAR_WID)
    ompl = OMPL(plot_bounds, car, parking_lot_bitmap)

    
    
    colours = {"bicycle": "deepskyblue", "4ws": "orange"}
    results = {m: ompl.create_planner(m) for m in ("bicycle", "4ws")}

    plot = Plot(parking_lot_bitmap, results, plot_bounds, colours, car)
    plot.static_plot(os.path.join(os.getcwd(), "src/parking/parking_paths.png"))
    plot.animate(os.path.join(os.getcwd(), "src/parking/parking_animation.gif"))


if __name__ == "__main__":
    main()