# Gazebo Auto Simulation + Tracker Test (ROS2 Humble)

End-to-end **simulator integration** to showcase automatic testing of your trackers:
- Launches **Gazebo** world and spawns multiple moving targets.
- Publishes **noisy detections** (PoseArray) to feed your tracker (e.g., EKF-JPDA).
- Collects **ground truth** and **tracker outputs**, computes simple metrics, and writes **CSV** in `results/`.

> Works standalone. If you also clone **EKF-JPDA-Tracker**, this launch will wire detections → tracker → metrics automatically.

---

## Quickstart

```bash
colcon build --symlink-install
source install/setup.bash

# Start full stack: Gazebo + spawner + mover + detector + evaluator
ros2 launch sim_autotest bringup.launch.py targets:=5 duration:=60.0 noise:=0.25

# After run, check metrics
ls results/
cat results/metrics.csv
```
If you have the EKF-JPDA tracker available in the same workspace, pass `use_tracker:=true` to include it:
```bash
ros2 launch sim_autotest bringup.launch.py use_tracker:=true
```

---

## Nodes

- `spawner.py` – uses `gazebo_ros`'s `spawn_entity.py` to place spheres.
- `mover.py` – simulates target kinematics and publishes **ground truth** `/gt/poses` (PoseArray).
- `detector.py` – adds Gaussian noise + false positives/negatives to produce `/detections2d` (PoseArray).
- `evaluator.py` – matches tracks to GT and writes **MAE**, **ID switches**, and **coverage** to `results/metrics.csv`.

---

## Topics
- **Ground truth**: `/gt/poses` (`geometry_msgs/PoseArray`)
- **Detections**: `/detections2d` (`geometry_msgs/PoseArray`) — intentionally the same type used by EKF-JPDA’s input
- **Tracker states** (if available): `/tracks/states` (`std_msgs/String` JSON)

---

## Config params
- `targets` (int): number of simulated targets (default 5)
- `duration` (float): seconds to run before the evaluator stops and writes results (default 60)
- `noise` (float): detection noise std in meters (default 0.25)
- `fp_rate` (float): false positive ratio per frame (default 0.1)
- `fn_rate` (float): false negative probability per detection (default 0.05)
