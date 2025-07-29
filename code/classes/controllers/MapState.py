class MapState():
    def __init__(self):
        self.locations = {}

    def add_location(self, name, position):
        self.locations[name] = position,

    def get_location(self, name):
        return self.locations.get(name)

    def get_all_locations(self):
        return self.locations
