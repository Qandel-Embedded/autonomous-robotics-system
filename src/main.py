"""Main robot controller — integrates sensor fusion, planning, and control."""
import numpy as np
from sensor_fusion import SensorFusion
from path_planner import AStarPlanner, cells_to_meters
from motor_controller import DifferentialDriveController


def main():
    fusion     = SensorFusion()
    controller = DifferentialDriveController()

    # Simple 10x10 grid with some obstacles
    grid = np.zeros((10, 10), dtype=int)
    grid[3, 2:7] = 1   # Wall obstacle

    planner = AStarPlanner(grid)
    start, goal = (0, 0), (9, 9)
    path = planner.plan(start, goal)
    waypoints = cells_to_meters(path, planner.res)

    print(f"[ROBOT] Path found: {len(waypoints)} waypoints")

    for wp in waypoints:
        px, py = fusion.update(imu_ax=0.0, imu_ay=0.0, enc_vx=0.5, enc_vy=0.0)
        v_left, v_right = controller.set_velocity(linear_mps=0.3, angular_rads=0.0)
        print(f"  → pos=({px:.3f},{py:.3f})  L={v_left:.1f}  R={v_right:.1f}")


if __name__ == "__main__":
    main()
