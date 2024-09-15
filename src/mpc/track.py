from path import Path

class Track:
    def __init__(self):
        self.left_bound = None # Set to tck of spline from left bound (load from file, or object, or smth like that dk yet)
        self.right_bound = None # right bound as above
        self.optimal_path = None # same but optimal path
        # and so on..
    