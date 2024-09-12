from path import Path

class VehicleBase:
    """Vehicle parameters and behaviour."""

    def __init__(self):
        pass

    def load_params(self, filepath):
        pass

    def get_velocity_profile(self, path: Path, samples, is_closed):
        pass
