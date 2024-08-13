from src.robot import Robot
from src.point import Point
from src.planner import Planner

def main():
    # Initialize robots at different starting positions with different priorities
    robot1 = Robot(Point(0, 0), "Robot 1", priority=2)  # Higher priority
    robot2 = Robot(Point(9, 9), "Robot 2", priority=1)  # Lower priority

    # Define the grid size and obstacles
    grid_size = 10
    obstacles = {
        Point(0, 2), Point(0, 6), Point(1, 1), Point(1, 3), Point(3, 2),
        Point(6, 1), Point(7, 3), Point(7, 9), Point(9, 1)
    }

    # Define the drop-off locations
    drop_off_locations = [Point(0, 9), Point(5, 5)]

    # Initialize the central planner with the robots, grid size, and drop-off locations
    central_planner = Planner([robot1, robot2], grid_size, drop_off_locations, obstacles)

    # Add some dig locations
    central_planner.add_dig_location(Point(1, 7))
    central_planner.add_dig_location(Point(7, 4))
    central_planner.add_dig_location(Point(8, 1))

    # Assign tasks to robots and calculate the completion time
    central_planner.assign_tasks()

    # Assign tasks to robots and calculate the completion time
    estimated_time = central_planner.get_estimated_completion_time()
    print(f"Estimated completion time: {estimated_time} ticks")

    # Plot the robots' paths
    central_planner.plot_paths()

    # Generate and plot the grids at different stages
    central_planner.generate_plots()

    # Display the detailed path log for each robot
    for robot in [robot1, robot2]:
        print(f"{robot.name} Path Log: {' -> '.join(robot.path_log)}")

if __name__ == "__main__":
    main()
