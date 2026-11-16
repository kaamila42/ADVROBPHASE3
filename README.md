# Phase 3: RoboMaster wheel odometry and IMU fusion

This submission contains the Python implementation of a ROS 2 package for estimating a RoboMaster EP robot's planar motion and approaching an AprilTag. The main method predicts motion using wheel measurements and corrects the estimate using an independent IMU gyroscope measurement.

## Important Python files

All application modules are in `rm_phase3_odometry/`.

| File | Purpose |
| --- | --- |
| `nav_node.py` | Main ROS node: receives sensor data, runs estimation, steers the robot, applies motion checks and records trials. |
| `state_estimator.py` | Five-state extended Kalman filter (EKF), covariance prediction, gyro correction and innovation gating. |
| `motion_model.py` | Planar motion equations and their Jacobians, including wheel-driven prediction. |
| `sensor_measurements.py` | Wheel kinematics, timestamp checks, front-guard displacement and final-rest detection. |
| `imu_processing.py` | Gyroscope validation and stationary bias calibration. |
| `camera_node.py` | Separate camera process: receives images and calibration, detects the target and publishes observations. |
| `tag_detector.py` | OpenCV AprilTag detection and optional calibrated pose estimation. |
| `trial_logger.py` | Writes telemetry, metadata and trial summaries. |
| `trial_analysis.py` | Enters independent distance measurements, summarizes errors and generates plots. |
| `movement_plots.py`, `estimator_plots.py` | Movement comparison and estimator diagnostic plots. |
| `offline_evaluation.py` | Historical-data audit and synthetic estimator comparisons. |
| `video_diagnostics.py` | Optional passive camera delivery/timing diagnostic. |

The remaining Python modules are retained because they support the core nodes and analysis commands. `setup.py`, `setup.cfg`, `package.xml` and `resource/` provide ROS package installation; `launch/` and `config/` provide launch scripts and parameter files.

## How it works

1. The driver publishes wheel measurements on `/joint_states_p`, IMU measurements on `/imu`, camera images and driver odometry.
2. Wheel kinematics convert measured wheel speeds into forward velocity and yaw rate. The EKF state is `[x, y, theta, v, omega]`, in metres, radians, metres/second and radians/second.
3. Wheel prediction integrates distance and heading using trapezoidal rates and midpoint heading. Its Jacobians propagate uncertainty: `P_pred = F P F.T + G Qwheel G.T + Qmodel * dt`.
4. After stationary gyro bias calibration, the independent Z-axis gyro corrects the yaw-rate state with observation matrix `H = [0, 0, 0, 0, 1]`. The filter gates inconsistent measurements using normalized innovation squared and updates covariance in Joseph form.
5. The camera node detects AprilTag family `tag36h11`, ID 3, and sends the bearing and image-width fraction to navigation. Navigation uses bearing feedback to steer toward the tag and stops when the tag width reaches 20% of the image width. This is an image-based arrival rule, not a direct metric distance measurement.
6. Trial logging records sensor data, estimates, uncertainty, commands and correction diagnostics. Final distance-error analysis uses an independently measured physical distance and confirmed final rest.

The normal experiment starts 2.00 m from the robot's front guard; the configured physical target is 0.26 m. The 0.20 m nominal stopping-distance setting is a separate lost-tag travel safeguard.

## Estimation modes

| Parameter value | Method |
| --- | --- |
| `wheel_only` | Wheel-based prediction without gyro correction. |
| `odom_velocity` | Earlier constant-velocity EKF with driver-odometry velocity correction. |
| `wheel_imu` | Default: wheel prediction plus independent gyro correction. |

In the raw wheel modes, driver `/odom` is diagnostic: its angular velocity already includes the gyro, so treating it as another independent gyro observation would double-count information. Accelerometer measurements are diagnostic and are not integrated. Camera metric range and ToF position fusion are not part of the default wheel/IMU estimator.

## Build and run

Requires ROS 2 Jazzy, colcon, the compatible `robomaster_ros` driver and `robomaster_msgs`, NumPy, OpenCV with `cv2.aruco` AprilTag support, PyYAML and Matplotlib. ROS dependencies are declared in `package.xml`.

For a fresh workspace, copy this folder into its `src/` directory and name it `rm_phase3_odometry`. Avoid adding a second copy of the package to an existing workspace.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/phase3_submission_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select rm_phase3_odometry
source install/setup.bash
```

The launch scripts default to `~/rm_venv/bin/python3`. Supply an interpreter with both ROS and application dependencies if that path does not exist. With one compatible driver already running, launch without enabling chassis motion:

```bash
ros2 launch rm_phase3_odometry phase3.launch.py \
  start_driver:=false enable_motion:=false estimation_mode:=wheel_imu \
  python_executable:=/usr/bin/python3 \
  output_dir:=$HOME/phase3_results
```

Set `view_camera:=false` on a machine without a desktop. `start_driver:=true` can start the driver instead; never run a second driver against the same robot. For that option, see `driver_patch/README.md`: the supplied low-latency video launch options require the compatible patched driver. Set `driver_python` to the appropriate interpreter as needed.

Before a supervised moving trial, copy `config/phase3_params.yaml` to a calibration file and supply measured sensor conventions and front-guard geometry. Motion is disabled by default; the verification flags and geometry placeholders are deliberately unset. Follow **Build and operator commands** in `EKF_CORRECTION.md` for calibration and the moving-trial command. The optional saved-bias, stationary-noise and target-width navigation switches remain off in the supplied defaults.

## Outputs and analysis

Each run writes a new trial directory containing telemetry, metadata and summaries. Plot generation can add movement and estimator diagnostic panels. Example commands after building and sourcing the package:

```bash
ros2 run rm_phase3_odometry trial_analysis plot /path/to/trial_directory
ros2 run rm_phase3_odometry trial_analysis measure /path/to/trial_directory 0.27
ros2 run rm_phase3_odometry trial_analysis summarize "$HOME/phase3_results" --plot
```

The `0.27` above is only a command example: replace it with the independently measured distance in metres for that trial. Command-integrated paths and driver odometry are comparison references, not independent ground truth.

## Scope and supporting documents

- `EKF_CORRECTION.md`: detailed equations, sensor interfaces, assumptions and calibration/operator instructions.
- `VERIFICATION.md`: original development validation record; its recorded checks were not rerun merely by packaging this submission.
- `driver_patch/`: optional companion-driver video patch and installation instructions.

This folder is a source-code submission. It excludes `.git`, videos, large result archives, historical trial data, generated plots and build caches. Historical result paths and workspace-specific statements in the supporting documents refer to the original development workspace. No new robot accuracy result is claimed by this package; historical logs lack the raw wheel data required to validate the revised wheel/IMU estimator retrospectively.
