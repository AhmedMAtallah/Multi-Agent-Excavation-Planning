from .point import Point

class Robot:
    def __init__(self, start_position, name, priority):
        self.initial_position = start_position
        self.position = start_position
        self.load = 0
        self.name = name
        self.priority = priority
        self.path_log = [f"({self.position.x},{self.position.y})"]
        self.drop_off_count = 0

    def move_to(self, new_position):
        self.position = new_position
        self.path_log.append(f"({self.position.x},{self.position.y})")
        print(f"{self.name} moved to position {self.position.x}, {self.position.y}")

    def dig(self):
        self.load = 1
        self.path_log.append("dig")
        print(f"{self.name} dug at position", self.position.x, self.position.y)

    def offload(self):
        self.load = 0
        self.drop_off_count += 1
        self.path_log.append(f"drop off ({self.drop_off_count})")
        print(f"{self.name} offloaded at drop-off location")
