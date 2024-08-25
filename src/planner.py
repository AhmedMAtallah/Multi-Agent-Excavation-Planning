import heapq
from collections import deque
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from .robot import Robot
from .point import Point

class Planner:
    def __init__(self, robots, grid_size, drop_off_locations, obstacles):
        self.dig_locations = deque()
        self.robots = sorted(robots, key=lambda r: r.priority)
        self.grid_size = grid_size
        self.drop_off_locations = drop_off_locations
        self.obstacles = obstacles
        self.paths = {robot.name: [robot.position] for robot in robots}
        self.dig_points = []
        self.drop_off_points = []
        self.robot_positions = {robot.name: robot.position for robot in robots}
        self.total_time = 0  # Initialize total_time here

    def add_dig_location(self, location):
        self.dig_locations.append(location)
        print(f"Added dig location at {location.x}, {location.y}")

    def a_star(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []
    
    def get_adjacent_position(self, drop_off_location, current_position):
        """Find a position adjacent to the drop-off location where the robot can offload."""
        candidates = [
            Point(drop_off_location.x + 1, drop_off_location.y),
            Point(drop_off_location.x - 1, drop_off_location.y),
            Point(drop_off_location.x, drop_off_location.y + 1),
            Point(drop_off_location.x, drop_off_location.y - 1)
        ]

        # Filter candidates to ensure they're within bounds and not blocked
        valid_candidates = [
            pos for pos in candidates
            if 0 <= pos.x < self.grid_size and 0 <= pos.y < self.grid_size and pos not in self.obstacles
        ]

        # Return the closest valid position
        return min(valid_candidates, key=lambda pos: self.heuristic(current_position, pos))

    def heuristic(self, point, goal):
        return abs(point.x - goal.x) + abs(point.y - goal.y)

    def get_neighbors(self, point):
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        neighbors = []
        for dx, dy in directions:
            neighbor = Point(point.x + dx, point.y + dy)
            if (0 <= neighbor.x < self.grid_size and 0 <= neighbor.y < self.grid_size and neighbor not in self.obstacles):
                neighbors.append(neighbor)
        return neighbors

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        total_path.reverse()
        return total_path
    
    def assign_tasks(self):
        self.total_time = 0 
        time_step = 0
        
        # Initialize reservations
        reservations = {time_step: set() for time_step in range(self.grid_size * 2)}

        while self.dig_locations:
            for robot in self.robots:
                if not self.dig_locations:
                    break

                current_position = robot.position
                next_location = min(self.dig_locations, key=lambda loc: self.heuristic(current_position, loc))
                self.dig_locations.remove(next_location)

                path_to_dig = self.a_star(current_position, next_location)

                for step in path_to_dig[1:]:
                    # Check reservation for the next time step
                    if step not in reservations[time_step + 1]:
                        robot.move_to(step)
                        self.paths[robot.name].append(step)
                        self.robot_positions[robot.name] = step
                        self.total_time += 1
                        reservations[time_step + 1].add(step)
                    else:
                        # Check which robot has higher priority
                        occupying_robot = next(
                            (r for r in self.robots if self.robot_positions[r.name] == step),
                            None
                        )
                        if occupying_robot and robot.priority >= occupying_robot.priority:
                            print(f"{robot.name} with lower priority treats position {step.x},{step.y} as an obstacle.")
                            self.robot_positions[occupying_robot.name] = robot.position
                            robot.move_to(step)
                            self.paths[robot.name].append(step)
                            self.robot_positions[robot.name] = step
                            self.total_time += 1
                            reservations[time_step + 1].add(step)
                        else:
                            print(f"{robot.name} with lower priority waits or recalculates.")
                            path_to_dig = self.a_star(robot.position, next_location)
                            break                

                        print(f"{robot.name} with lower priority treats position {step.x},{step.y} as an obstacle.")
                        self.obstacles.add(step)
                        path_to_dig = self.a_star(robot.position, next_location)
                        break

                self.dig_points.append(next_location)
                robot.dig()
                self.total_time += 1
                time_step += 1

                drop_off_location = min(self.drop_off_locations, key=lambda loc: self.heuristic(robot.position, loc))
                adjacent_position = self.get_adjacent_position(drop_off_location, robot.position)

                path_to_drop_off = self.a_star(robot.position, adjacent_position)

                for step in path_to_drop_off[1:]:
                    # Check reservation for the next time step
                    if step not in reservations[time_step + 1]:
                        robot.move_to(step)
                        self.paths[robot.name].append(step)
                        self.robot_positions[robot.name] = step
                        self.total_time += 1
                        reservations[time_step + 1].add(step)
                    else:
                        print(f"{robot.name} with lower priority treats position {step.x},{step.y} as an obstacle.")
                        path_to_drop_off = self.a_star(robot.position, adjacent_position)
                        break

                self.drop_off_points.append(drop_off_location)
                robot.offload()
                self.total_time += 1
                time_step += 1

    def get_estimated_completion_time(self) -> int:
        """Return the total time computed during assign_tasks."""
        return self.total_time

    def is_position_occupied(self, position, current_robot):
        for robot_name, robot_position in self.robot_positions.items():
            if robot_position == position and current_robot.priority >= next(
                r.priority for r in self.robots if r.name == robot_name
            ):
                return True
        return False

    def plot_paths(self):
        fig, ax = plt.subplots()
        ax.set_xlim(-0.5, self.grid_size - 0.5)
        ax.set_ylim(-0.5, self.grid_size - 0.5)

        for i in range(self.grid_size):
            ax.axhline(i, color='gray', linewidth=0.5)
            ax.axvline(i, color='gray', linewidth=0.5)

        for robot in self.robots:
            x_coords = [point.x for point in self.paths[robot.name]]
            y_coords = [point.y for point in self.paths[robot.name]]
            ax.plot(y_coords, x_coords, marker="o", label=f"{robot.name} Path")

        dig_x = [loc.y for loc in self.dig_points]
        dig_y = [loc.x for loc in self.dig_points]
        ax.scatter(dig_x, dig_y, marker="o", color="blue", s=200, label="Dig Locations")

        drop_off_counts = {loc: 0 for loc in self.drop_off_locations}
        for drop_off in self.drop_off_points:
            drop_off_counts[drop_off] += 1

        for loc, count in drop_off_counts.items():
            if count > 0:
                ax.text(loc.y, loc.x, str(count), fontsize=12, ha='center', va='center', color='green', fontweight='bold')

        for obstacle in self.obstacles:
            ax.text(obstacle.y, obstacle.x, 'X', fontsize=20, ha='center', va='center', color='red', fontweight='bold')

        ax.set_xlabel('Horizontal (y)')
        ax.set_ylabel('Vertical (x)')
        ax.xaxis.set_label_position('top')
        ax.xaxis.tick_top()

        plt.gca().invert_yaxis()
        plt.legend(loc='upper left', bbox_to_anchor=(1, 1))
        plt.grid(True)
        plt.savefig('output/paths.png')

    def generate_grid(self, with_digs=False, with_final_positions=False):
        grid = [['.' for _ in range(self.grid_size)] for _ in range(self.grid_size)]

        for obs in self.obstacles:
            grid[obs.x][obs.y] = '#'

        for i, drop_off in enumerate(self.drop_off_locations):
            grid[drop_off.x][drop_off.y] = 'O' if not with_final_positions else str(i + 1)

        if not with_final_positions:
            for robot in self.robots:
                grid[robot.initial_position.x][robot.initial_position.y] = 'R'
        else:
            for robot in self.robots:
                grid[robot.position.x][robot.position.y] = 'R'

        if with_digs or with_final_positions:
            for dig in self.dig_points:
                grid[dig.x][dig.y] = 'D'

        return grid

    def plot_grid(self, grid, ax, title):
        ax.set_xlim(-0.5, self.grid_size - 0.5)
        ax.set_ylim(-0.5, self.grid_size - 0.5)
        ax.set_title(title)

        for i in range(self.grid_size):
            ax.axhline(i, color='gray', linewidth=0.5)
            ax.axvline(i, color='gray', linewidth=0.5)

        for x in range(self.grid_size):
            for y in range(self.grid_size):
                ax.text(y, x, grid[x][y], fontsize=20, ha='center', va='center', color='green' if grid[x][y] in ['R', 'R1', 'R2', '1', '2'] else 'black')

        ax.invert_yaxis()

    def generate_plots(self):
        initial_grid = self.generate_grid()
        grid_after_digs = self.generate_grid(with_digs=True)
        final_grid = self.generate_grid(with_final_positions=True)

        fig, axes = plt.subplots(1, 3, figsize=(18, 6))
        self.plot_grid(initial_grid, axes[0], "Initial grid")
        self.plot_grid(grid_after_digs, axes[1], "Grid after sending dig locations")
        self.plot_grid(final_grid, axes[2], f"Final grid after tasks\n(Total ticks: {self.total_time})")

        plt.savefig('output/grid_before_and_after.png')
