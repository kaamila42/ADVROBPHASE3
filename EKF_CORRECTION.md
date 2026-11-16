# Phase 3: wheel-odometry prediction and independent IMU correction

The assignment PDF `~/Downloads/RoboMaster_Odometry_Project_Phase_3(1).pdf` requires: “Apply odometry-based prediction and sensor-based correction to estimate the robot’s position during the motion.” Mode C now implements **raw ESC wheel prediction → independent gyro correction**. The earlier IMU-prediction/wheel-correction revision is superseded and removed. The existing five-state EKF correction, numerical solves, Joseph update and legacy process model are retained.

No robot driver or physical trial was started during development. Default `enable_motion: false` remains. No Git push was made. All original experimental files and Phase 2 remain unchanged.

## Original problem and modes

Originally the filter predicted using previous velocity states and zero acceleration input, then corrected from driver `/odom` velocities. Raw IMU was diagnostic. That is retained as Mode B for historical reproduction, but does not explicitly separate independent wheel prediction from gyro correction.

| Mode | `estimation_mode` | Prediction | Correction |
|---|---|---|---|
| A | `wheel_only` | Raw wheel odometry | None |
| B | `odom_velocity` | Original Lecture 11A model, zero mean acceleration | Original driver `/odom` v/omega |
| C (recommended after calibration) | `wheel_imu` | Raw wheel odometry, covariance from wheel uncertainty | Independent bias-compensated raw gyro |

The same navigation controller, tag detector, speed limits and width arrival condition are used for every mode. No automatic fallback or command-speed substitution exists. Unrecognized modes fail initialization. The old, reversed `raw_wheels_imu` mode name is rejected rather than silently changing its meaning.

## Verified interfaces and independence

The installed driver egg-link resolves through `~/ros2_ws/build/robomaster_ros` to `~/ros2_ws/src/robomaster_ros/robomaster_ros/robomaster_ros/`. The installed and inspected `modules/chassis.py` are the same file (verified path/hash).

| Topic, default unprefixed | ROS message type | Actual source and use |
|---|---|---|
| `/joint_states_p` | `sensor_msgs/msg/JointState` | `client.py` publisher; `Chassis.updated_esc` converts SDK wheel RPM to rad/s; prediction in A/C |
| `/imu` | `sensor_msgs/msg/Imu` | `Chassis.updated_imu`; gyro correction only in C, diagnostic in A/B |
| `/odom` | `nav_msgs/msg/Odometry` | SDK position, attitude, velocity and the same gyro; comparison/existing safety in A/C; original update in B |
| `/range_0` | `sensor_msgs/msg/Range` | `modules/tof.py`, millimetres converted to metres, `tof_0_link`; optional verified geometry only |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Existing chassis velocity command subscription; never an estimator measurement |

Driver details:

* `updated_position` uses SDK `sub_position(cs=1)`: X unchanged, Y negated. Its firmware fusion is not disclosed.
* `updated_attitude` uses SDK yaw/pitch/roll with yaw and pitch sign conversions. Driver pose is not raw encoder odometry.
* `updated_velocity` defaults to SDK body velocity entries 3/4, with Y negated. A configurable pose-difference variant exists; internal SDK velocity fusion is not exposed.
* `publish_state_estimation` directly assigns `imu_msg.angular_velocity` to `/odom.twist.angular`. It would double-count gyro evidence to use this as an independent wheel input and then fuse raw gyro.
* `updated_imu` converts accelerometer g to m/s² using 9.81 and signs `(1,-1,-1)`, and applies the same signs to gyro floats. The installed SDK rounds gyro values without unit conversion. ROS declares rad/s; verify physical scale and positive yaw direction before enabling motion.
* `updated_esc` uses FR, FL, RL, RR and signs `(1,-1,-1,1)` for wheel joint rates. Its input is independent SDK ESC speed, not driver odometry. Wheel positions wrap; this implementation uses measured angular velocities rather than wrapped angle differences.
* Both wheel kinematics and the published gyro use the verified ROS `base_link` body convention. The gyro preprocessor requires that exact frame; unknown mounts/prefixes fail safely. It does not assume an arbitrary `imu_link` shares the wheel axes.
* `/joint_states_p` also contains non-wheel joints. These are ignored; all four exact wheel names must be present in a wheel message. No `/joint_states` aggregation or invented topic is used.

Driver ROS stamps are receipt/publish times, not original hardware acquisition times. IMU messages are published on position callbacks and can contain cached gyro payloads. Equal payloads cannot reliably distinguish cached data from legitimate repeated readings. Unique timestamps, matched rates, conservative covariance and hardware timing checks mitigate but do not eliminate this driver limitation.

## Wheel kinematics

Read Lecture11B slide 7, the installed driver's `wheel_speeds_from_twist`, and `robomaster_description/urdf/base.urdf.xacro`. The wheel radius is **0.05 m**; wheel centers are `(±0.1,±0.1)` m. The driver's `AXIS=lx+ly=0.20 m`. The base-to-chassis fixed transform has only Z translation.

`wheel_velocity` orders joint names as FR, FL, RL, RR, independently of array order. Undo the driver's ESC signs `(1,-1,-1,1)` once to obtain forward-positive rolling angular rates q1..q4, rad/s:

```
v_w = .05/4 * (q1+q2+q3+q4)
vy  = .05/4 * (q1-q2+q3-q4)
w_w = .05/(4*.20) * (q1-q2-q3+q4)
```

This is the Lecture 11B mecanum arrangement mapped to driver numbering, not differential-drive kinematics. Lateral speed above the existing safety limit is rejected. Tests cover shuffled names, positive/reverse translation and signed pure rotation. Physical direction and rolling scale still require verification on this robot.

## Actual Mode A/C transition and Jacobians

The state remains `[px,py,theta,v,omega]` with units `[m,m,rad,m/s,rad/s]`. The original acceleration-input Lecture 11A equations and F/G remain unchanged in `motion_model` / `jacobians` for Mode B and lecture tests.

For A/C, the measured wheel input is `u=[v_w,w_w]`. To retain the influence of corrected omega on subsequent heading, integrate the previous corrected rate and current measured wheel endpoint using trapezoidal velocities and midpoint pose:

```
d = dt/2
s = d*(v + v_w)
h = d*(omega + w_w)
phi = theta + h/2

px_minus    = px + s*cos(phi)
py_minus    = py + s*sin(phi)
theta_minus = wrap(theta + h)
v_minus     = v_w
omega_minus = w_w
```

This is a wheel-driven numerical discretization of the lecture unicycle. It differs explicitly from using `s=v_w*dt` and `h=w_w*dt`: that pure overwrite would discard corrected omega's influence on the next heading. Here corrected omega contributes through `h`, corrected heading persists, and both influence later position. No gyro is inserted as a prediction input and no noisy wheel derivative is computed. A uses the identical transition without any correction.

With `c=cos(phi)`, `t=sin(phi)` and the state/input orders above:

```
F = [[1,0,-s*t, d*c, -s*t*d/2],
     [0,1, s*c, d*t,  s*c*d/2],
     [0,0,   1,   0,        d],
     [0,0,   0,   0,        0],
     [0,0,   0,   0,        0]]

G = [[d*c, -s*t*d/2],
     [d*t,  s*c*d/2],
     [  0,        d],
     [  1,        0],
     [  0,        1]]

P_minus = F*P_plus*F.T + G*Qwheel*G.T + Qmodel_density*dt
```

Endpoint v/omega rows are overwritten by the current independent wheel sample, which is why their old-state derivatives are zero. G puts current wheel uncertainty directly into pose and rate covariance, creating pose/rate cross-covariances. The previous endpoint's uncertainty is already carried by P. First wheel input initializes the rates and their Qwheel block without moving position; subsequent intervals must be positive and <=max_dt. Initial wheel motion is rejected. Central finite differences verify both F/G and the original lecture F/G.

## Independent gyro correction

`gyro_measurement` accepts the verified driver's body gyro frame, finite rates and plausible bounds. On this level planar experiment, select base Z. SDK sign conversion has already happened in the driver; it is not applied twice. No SDK attitude quaternion or accelerometer is used by the filter. The accelerometer and quaternion remain raw diagnostics. Tilted/nonplanar operation requires a verified orientation/axis model; this five-state implementation assumes a level arena and does not claim general 3D estimation.

Startup `GyroBias` collects 50 consecutive samples while fresh ESC rates on **all four wheels** are below .05 rad/s. Motion clears the batch. Excessive gyro mean/scatter rejects calibration. Bias is frozen during the trial; physically keep the robot still, since wheel stillness cannot detect carrying/sliding. Calibration samples are not also applied as independent correction observations.

```
z = raw_gyro_z - stationary_bias_z
h(x) = omega
H = [[0,0,0,0,1]]
e = z - H*x_minus
S = H*P_minus*H.T + R
K = solve(S, H*P_minus).T
x_plus = x_minus + K*e
A = I-K*H
P_plus = A*P_minus*A.T + K*R*K.T
NIS = e.T * solve(S,e)
```

Heading is normalized, covariance symmetrized and PSD checked. NIS gate defaults to 9 for the scalar gyro. Rejected updates leave state/P unchanged and stop an active C trial. Full covariance is retained. No gain tuning or explicit inverse is used. Gyro correction can move pose through cross-covariance and corrected angular rate affects the following prediction.

### Synchronization and dropouts

Wheel messages drive prediction. A pending gyro is corrected at the first wheel epoch at or after its ROS stamp with skew <=0.08 s. Same-stamp callbacks work in either arrival order. A future gyro waits; stale/too-old gyro is skipped. Each gyro stamp is consumed once, with at most one correction per wheel epoch. There is no re-prediction at an IMU callback or rewind. Skew uncertainty adds `(skew*4 rad/s²)^2` to R. This is an explicitly approximate receipt-time association, not an exact acquisition-time filter.

Wheel gaps larger than max_dt are rejected rather than bridged with invented motion. C motion requires fresh wheels, valid gyro, completed bias calibration and a recent accepted synchronized correction; sensor dropouts fail safely. A requires wheels, but no gyro. B never uses raw gyro as a second filter input. `/odom` never predicts or corrects A/C, although its existing navigation-health checks are retained.

## Covariance assumptions and limitations

* Original P0 diagonal: `[.0001,.0001,.001,.01,.04]` in squared state units. Pose origin is local, not an absolute map reference. A/C initialize rate covariance with Qwheel on their first sample.
* Separate wheel-input Qwheel diagonal: `[.0025,.01]`, units `(m/s)^2,(rad/s)^2`, standard deviations .05 m/s and .1 rad/s.
* Qmodel density retains `[.0001,.0001,.0001,.0025,.01]` squared state units per second; multiplied by dt. It represents unmodeled motion and residual effects conservatively.
* Gyro R: maximum of declared driver Z variance and configurable .01 `(rad/s)^2`, plus stationary sample-mean bias variance and skew uncertainty. Zero/unknown driver covariance does not imply perfect sensing.
* B retains input Qu `[.25,1.]` in acceleration/angular-acceleration variance units, original Qmodel and driver velocity covariance plus existing `[.0025,.01]` floors. These B tuning parameters are separate from A/C wheel Q.
* Driver setters square defaults .005 m/s and .03 rad/s, giving .000025 and .0009 variances, despite misleading parameter descriptor wording.

These conservative assumptions were not reduced to manufacture an improvement. Historical logs lack independent ESC data for empirical wheel variance or scale calibration. Stationary gyro scatter estimates white-noise bias uncertainty, but frozen bias errors and cached/colored samples can remain temporally correlated; a five-state filter does not explicitly estimate gyro bias drift. Q/R are engineering approximations requiring stationary and rotation/rolling calibration and inspection of NIS/gains.

Gyro does not observe absolute forward distance. Wheel slip, wrong wheel radius/scale and common drift cannot be reliably eliminated using gyro alone. A zero innovation or small covariance does not prove accurate distance.

## Optional external position observations

The existing OpenCV detector does calibrated `solvePnP` using camera intrinsics and configured printed tag size. `camera_node` can transform `tvec` using camera-to-base TF when arm pose is verified. However, a surveyed printed size, camera/guard/tag-plane geometry and calibrated range covariance have not been established here. Its optional metric range remains diagnostic; the 20% bounding box is never converted into a guessed metric position observation.

The existing ToF correction framework remains available in B only, behind its original beam/surface/offset/initial-reference checks. It defaults off and is not needed by A/C or AprilTag stopping. A/C reject enabling this legacy optional correction rather than applying an unverified model. Neither camera nor ToF position fusion is compulsory or claimed calibrated.

## Navigation, stopping and physical reference

Navigation is preserved: OpenCV AprilTag `tag36h11`, ID 3, camera calibration and diagnostics, straight approach, bearing steering, existing chassis limits, AP connection, tag-loss behavior and movement plots. No planner, search, tape following or Phase 2 mission changes.

* Starting distance: **2.00 m**, measured guard-to-tag plane.
* Normal arrival: **width fraction >=.20**, unchanged.
* Physical camera-based stopping target: **.26 m**, separately documented as `physical_target_distance_m`.
* `nominal_stopping_distance_m=.20` remains the original **lost-tag safeguard**, not the calibrated arrival target. The fallback safety calculation retains the original base displacement. Safety-related numerical values were not silently changed.

For unchanged heading, a fixed base/guard offset cancels: remaining distance is `d0 - ((p-p0) dot [cos(theta0),sin(theta0)])`. Do not subtract 45 mm. The URDF specifies front wheel center X=.1 m but does not identify a surveyed front-guard frame. Axle-to-guard 45 mm is not assumed to be base_link-to-guard.

For a measured guard offset o, use `p_guard=p+R(theta)*o`, and project guard displacement along the initial heading. The covariance uses the corresponding heading derivative via `guard_h`. Unknown geometry plus heading change yields NaN for primary guard remaining distance. Raw-mode motion remains gated by `guard_offset_verified` and physical start-reference verification; the default `[0,0]` is an unknown placeholder, not a claimed measurement.

### Verified final rest

The first stop command records its actual command publication stamp. Estimator callbacks continue during braking/settling. Distinct fresh wheel samples must keep all wheel rolling speeds below the configured linear threshold and wheel yaw below the angular threshold; C also checks corrected gyro rate for stillness. B uses the original measured odometry velocity. Defaults: .005 m/s, .03 rad/s, .5 s settling, 2 s timeout. Repeats cannot advance the interval, invalid/reversed/gapped readings reset it.

Telemetry identifies approach, braking, settling and confirmed stationary phases. Summary stores first-stop stamp, confirmed-stationary sensor stamp and final estimator stamp. Only a confirmed rest with a fresh valid estimator (and recent accepted gyro in C) gets primary `estimated_final_distance_m`. Otherwise record `unconfirmed_last_distance_m` and `final_estimate_valid=false`; manual error entry rejects an unconfirmed final estimate. The physical measurement is made afterward and entered separately.

```
estimation_error = estimated_final_guard_distance - measured_final_guard_distance
MAE = mean(abs(estimation_error))
stopping_target_error = measured_final_guard_distance - .26
```

Aggregation provides signed mean/min/max/span, sample standard deviation, MAE, per-mode estimation metrics, mean physical stopping distance, physical-distance sample standard deviation, mean deviation and MAE relative to .26 m. Stopping accuracy is separate from estimation accuracy. Do not pool the per-mode comparison; use `by_mode` in the JSON.

## Telemetry and plots

Each new invocation creates its own `results/trial_NNN_<UTC>/telemetry.csv`, metadata and summary; files are not reused. Raw and processed gyro, gyro bias/validity/consumed timestamp, raw wheel rates, wheel-derived rates/displacement, complete prior/posterior state/P, scalar gyro K/innovation/NIS/acceptance and final rest fields are logged throughout motion. Raw acceleration remains diagnostic; processed acceleration/angular-acceleration fields stay NaN. Unavailable values are blank/NaN. All quantities use SI; covariance indices follow the unit-labeled state names in metadata and CSV aliases.

Original movement figures remain under `trial_.../plots/<timestamp>/`. The plot command also writes `trial_.../estimator_plots/<timestamp>/estimator_validation.png` with nine panels: raw wheel displacement, predicted/corrected displacement, wheel versus corrected heading, wheel versus gyro rate, predicted/corrected remaining distance, gyro innovation, all five gains and position covariance. Unavailable series are explicitly labeled, not synthesized. Plots are refreshed after finalization; arrival-time plots are provisional.

## Historical audit and offline validation

The eight original physical distances remain **28,29,27,27,26,29,27,27 cm**. All original results are protected by a SHA-256 inventory. The read-only audit reproduces the last-row baseline:

| Method | MAE | Signed error min..max |
|---|---:|---:|
| Driver odometry baseline (not raw Mode A) | 8.898 cm | -10.549..-5.837 cm |
| Original EKF B | 23.168 cm | -25.220..+52.850 cm |

Only trial 004 has a verified stationary interval under the recorded original thresholds. The others are retained baseline statistics, not silently reclassified as confirmed final-rest errors. The physical mean stopping distance is .275 m, mean deviation from target +.015 m, sample standard deviation .01069 m; those independent measurements do not validate estimator performance by themselves.

Historical gyro values exist, but **raw ESC wheel rates are missing**. Thus A/C cannot be faithfully replayed on those eight trials. IMU orientation is not needed by this revised planar gyro-only correction. Driver-native acquisition times are also absent. No wheels are reconstructed from fused `/odom`; no revised physical MAE is invented.

Current derived output: `validation/wheel_prediction_imu_correction/` contains read-only historical CSV/JSON, eight baseline time plots, eight-trial final comparison, and explicitly synthetic A/B/C CSV/plots. Empty A/C historical results mean unavailable, not zero. The earlier `validation/ekf_correction/` is from the superseded reverse architecture and is not the current estimator evaluation.

Deterministic synthetic streams test acceleration, turning, braking and rest. A/B/C receive identical corresponding sensor samples; B's synthetic driver angular rate shares the simulated gyro just as the real driver does. Current final signed errors (synthetic only) are approximately A=-.003418 m, B=-.005515 m, C=-.003733 m. These numerical examples are not physical trials or evidence of improved live control; gyro cannot fix arbitrary forward slip. No measured improvement is claimed.

Run a new read-only evaluation outside historical results:

```bash
cd ~/ros2_ws/src/rm_phase3_odometry
~/rm_venv/bin/python3 -m rm_phase3_odometry.offline_evaluation results validation/new_wheel_gyro_audit
```

Tests cover wheel conversion/sign/order/pose, both model Jacobians, wheel-input pose uncertainty, gyro bias and correction, gain/innovation/Joseph covariance, cross-covariances, corrected-rate effect on subsequent pose, angle wrap, timestamp association/repeats/dropouts/gating, numerical stability, no gyro/odom double-counting, all modes' unchanged width controller, CSV generation, final-rest comparison, error/stopping metrics and validation plots. See `verification.txt` beside current validation output for final run counts/build/integrity checks.

## Build and operator commands

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select rm_phase3_odometry
source install/setup.bash
cd src/rm_phase3_odometry
~/rm_venv/bin/python3 -m pytest -q
```

Observation only; uses an already manually started driver:

```bash
ros2 launch rm_phase3_odometry phase3.launch.py \
  start_driver:=false enable_motion:=false estimation_mode:=wheel_imu start_distance_m:=2.0
```

Modes A/B substitute `estimation_mode:=wheel_only` or `estimation_mode:=odom_velocity`; the stopping controller stays the same.

Prepare a calibration file without guessing geometry:

```bash
cp ~/ros2_ws/src/rm_phase3_odometry/config/phase3_params.yaml ~/phase3_calibrated_params.yaml
```

After physical verification, edit that file to set `sensor_conventions_verified: true`, measured `guard_offset_xy_m`, and `guard_offset_verified: true`. Verify gyro scale/sign, level operation, stationary bias, actual wheel forward/rotation signs, radius/rolling scale, timestamp association, camera/arm setup and 2.00 m guard starting reference. These flags remain false in shipped defaults. A requires wheel convention verification; C additionally needs completed gyro calibration and accepted synchronized corrections. No operator confirmation is bypassed by the launch below.

One supervised trial, only after these checks and with the existing driver running:

```bash
ros2 launch rm_phase3_odometry phase3.launch.py \
  params_file:=$HOME/phase3_calibrated_params.yaml \
  start_driver:=false enable_motion:=true estimation_mode:=wheel_imu \
  start_reference_verified:=true start_distance_m:=2.0
```

Optional existing `start_driver:=true` retains AP-mode settings; it was not executed during development. Do not use the launch as a substitute for manual supervision. After confirmed rest, enter a **new** trial's independent measured metres:

```bash
~/rm_venv/bin/python3 -m rm_phase3_odometry.trial_analysis measure /path/to/new/trial_directory 0.27
~/rm_venv/bin/python3 -m rm_phase3_odometry.trial_analysis plot /path/to/new/trial_directory
```

For aggregation use a directory containing new measured trials or a separate copy, keeping original eight-trial metadata/plots/error tables untouched.

## Changed and preserved files

Changed within the existing package:

* `motion_model.py`: retain original lecture model/Jacobians; add consistent wheel midpoint transition/F/G.
* `state_estimator.py`: add wheel covariance prediction and scalar gyro correction; retain original prediction and solve/Joseph framework.
* `imu_processing.py`: gyro frame/axis validation and frozen stationary bias; no acceleration integration.
* `sensor_measurements.py`: verified wheel conversion, guard transform/Jacobian, robust settling.
* `nav_node.py`: A/B/C routing, wheel prediction, independent synchronized gyro correction, diagnostics and final-rest capture.
* `trial_logger.py`: additive wheel/gyro/prediction/correction/final fields and target metadata.
* `trial_analysis.py`: verified-rest error entry, per-mode metrics, separate stopping accuracy, automatic estimator plotting.
* `estimator_plots.py`: new nine-panel recorded-telemetry validation figure.
* `offline_evaluation.py`: read-only baseline audit and labeled synthetic A/B/C comparison.
* `config/phase3_params.yaml`, `launch/phase3.launch.py`: mode selection, 2.00 m start, separate physical target and original .20 m safeguard, safe calibration defaults; existing launch arguments retained.
* `tests/test_navigation.py`, `tests/test_raw_fusion.py`, `tests/test_trials.py`: explicit original-mode harness and current architecture validation.
* `README.md`, `VERIFICATION.md`, `EKF_CORRECTION.md`, `setup.py`: current architecture/operation documentation and installed guide.

Preserved: `tag_detector.py`, `camera_node.py`, `config/camera_calibration.yaml`, `movement_plots.py`, camera diagnostics, driver and robot description sources, AP connection, speed/steering limits, original historical CSV/metadata/plots/physical distances/error tables, and the entire Phase 2 `rm_flatness_nav` package. No separate robot project was created.
