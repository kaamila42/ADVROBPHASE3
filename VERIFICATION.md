# Current wheel-prediction / gyro-correction verification

Current architecture and test coverage: [EKF_CORRECTION.md](EKF_CORRECTION.md). Current machine-readable baseline/synthetic results and build/test/integrity evidence: `validation/wheel_prediction_imu_correction/`.

The original eight-trial baseline is preserved, not a replay of Mode C. Raw ESC inputs are missing from those historical logs; no revised physical accuracy claim is made. No hardware was started. Mode A uses wheels only, B preserves the original EKF, C predicts from wheels and corrects with independent gyro.

## Original verification notes (historical)

The notes below describe earlier package behavior; current mode/default/reference instructions are in the guide above.

> **Phase 3 EKF correction:** Current sensor architecture, 2.00 m guard reference, calibration gates, stopping semantics, offline results and commands are in [EKF_CORRECTION.md](EKF_CORRECTION.md). That document supersedes conflicting estimator/default/reference instructions below. Original historical results remain unchanged.

# Local verification record

Validation performed in the supplied Ubuntu workspace with ROS 2 Jazzy and Python 3.12. No robot motion or physical trials were performed.

| Check | Result |
|---|---|
| Lecture/assignment review | Read local PDFs, including specified lecture slides; assignment filename includes `(1)` |
| Phase 2 and driver inspection | Camera, calibration, odom, range, IMU, ESC, command, connection and arm interfaces inspected in installed source |
| Independent build | `colcon build --packages-select rm_phase3_odometry --symlink-install` passed |
| ROS test runner | `colcon test --packages-select rm_phase3_odometry` passed |
| Test results | 21 tests, 0 errors, 0 failures, 0 skipped |
| Process equations/Jacobians | Explicit numerical expectations and central finite differences passed |
| Covariance/correction | F/G propagation, PSD, velocity correction, off-diagonal position correction and Joseph covariance passed |
| Lecture 11B slide 15 | px=0.9555556 with covariance 0.08; px=1.0 with covariance zero |
| ToF | Displacement update, innovation rejection, negative-X forward travel, frame/range/time checks and geometry limits passed |
| Timestamps | Repeated, reversed, future, stale and unreasonable-gap rejection leaves EKF state/time unmodified |
| No IMU double-counting | Raw IMU callback changes diagnostics without changing x/P; unsupported raw-sensor modes rejected |
| Motion disabled | Actual node publishing method tested with an in-memory publisher: nonzero requested commands become zero Twist |
| CSV/analysis | Unique directories, initially blank independent measurement, untouched raw telemetry, known MAE/range/span and exclusion rules passed |
| Stopping | Arrival latch and stale camera/odom health checks passed; unverified final rest and estimator faults excluded from completed arrivals |
| Detector | Copied OpenCV detector recognizes a synthetic AprilTag ID 3 and produces bounding box/PnP; no claim of live-camera validation |
| Launch description | `ros2 launch rm_phase3_odometry phase3.launch.py --show-args` succeeded; driver and motion defaults are false |
| Startup/shutdown smoke check | 3-second motion-disabled process in isolated ROS domain 216 started and finalized on SIGINT; output under `/tmp/rm_phase3_startup_check`, not experimental results |
| Phase 2 integrity | SHA-256 checks passed for all inspected original Python modules, calibration YAMLs and launch file |
| Git | Separate local `main` repository initialized in new package; no remote created, no push |

The startup smoke command used `timeout --signal=INT --kill-after=2 3`, so its shell status 124 is expected; the process logged MANUAL_SHUTDOWN and wrote all three output files with the independent measurement blank. It launched no driver, SDK session or camera stream. DDS emitted socket permission errors in this environment. `ros2 topic list --no-daemon -t` could expose only its own infrastructure topics, not a verified live robot graph. Consequently live topic rates/types/frames, driver configuration service responses, actual marker performance, arm pose, ToF target surface/mounting offset, noise calibration and physical stop/accuracy must be verified on the robot using the README procedure before motion. Source-declared interfaces are distinguished from live verification.

The ToF correction path is implemented and tested but remains disabled until physical geometry and mounting are verified. No camera-range or raw-IMU fusion is implemented. No measured experimental errors, MAE or completed 8–10-trial result are claimed.

## Camera rollback and process separation

The camera regression changes were removed: navigation no longer decodes images, loads calibration YAML directly, adds calibration/dimension preview gates or requests TF at individual image timestamps. `camera_node.py` uses copied/adapted Phase 2 camera handling in its own process. AST comparisons confirmed unchanged `on_camera_info` and `quaternion_rotation`, and equivalent `camera_transform` after the timeout field rename. The detector matches Phase 2 except obsolete ToF braking overlay text. The image callback retains the original frame acceptance, 0.08 s throttle, detection, bearing, latest-TF pose and annotation calculations, replacing camera-side navigation/FSM effects with a timestamped application observation. The viewer again uses `/camera/annotated`. Phase 2 source checksums still pass.

Current build and colcon test pass: 29 tests, including copied CameraInfo calibration handling and observation payload, stale rejection, width arrival and immediate tag-loss zero-command behavior. Launch argument inspection succeeds. No robot or camera stream was launched during these checks, and no live frame-rate improvement is claimed. Use the launch file so both the camera process and estimator process run.


## Raw driver pipeline fix

The SDK receiver's full-queue path removes an old compressed chunk and omits the new chunk. An opt-in `robomaster_ros.video_pipeline.LosslessVideoConnection` replaces this receiver before the single existing camera stream starts. Byte-continuity backpressure tests, bounded-batch/sentinel tests, decoded newest-frame tests and full-queue shutdown tests pass. The actual driver decoder method was also exercised with fake decoded frames and no ROS node/network: receive timestamps and latest-frame replacement are correct. The shared driver's default feature flag is false; Phase 3 enables it explicitly. SDK package files and original Phase 2 package files were not changed. Driver changes, including a reproducible patch, are documented under `driver_patch/`.

Both packages build. Current Phase 3 suite: 38 tests passed. No live receiver was restarted and no robot command was sent. Hardware delivery improvements remain unverified. New per-stage driver counters will show whether any remaining gaps occur before TCP receipt, during decode, during publish or after publish at subscribers.


## CSV review, centering correction and revised arrival threshold

Reviewed existing trial 023 without modifying its data: 243 telemetry rows, populated command/odom/EKF/covariance records, old 0.17 width threshold, final estimated remaining distance 0.350385 m, independent measurement blank and final status ARRIVAL_REST_UNVERIFIED. Corrected covariance records are finite, symmetric and positive semidefinite. No experimental error was inferred without the independent measurement.

At the operator's request, new trials use width fraction 0.20 (128 px at 640 px image width) in navigation, overlay and metadata. This does not imply a calibrated physical distance of 20 cm. Centering gain/cap/minimum/deadband are now 1.6 / 0.30 rad/s / 0.05 rad/s / 0.01 rad. Missed detections hold the latest tag-directed heading rather than undoing camera corrections toward initial yaw. Tests cover the 17%-versus-20% boundary, steering sign/deadband/minimum/saturation and maintaining the tag-directed heading during missed detections. Both build and all 49 offline tests pass. Original Phase 2 checksums remain unchanged; no robot command was sent and no process restarted. Physical centering performance is not yet validated.

## Planned-versus-actual odometry plots

Added offline movement plotting and automatic post-finalization plotting in an independent process. "Planned" integrates recorded command Twists; "actual" is driver odometry in the initial forward/lateral coordinate frame. New telemetry records command-change events and odometry header times. Existing logs are explicitly labelled as sampled-command reference approximations.

Generated five PNGs, a 228-row derived comparison CSV and a reconstruction summary for trial 023, without changing the raw telemetry. All figure files were checked for existence/nonempty output, and the path plot was visually inspected. Tests cover exact straight/arc integration, command changes between odometry samples, negative global X, duplicate pose snapshots, missing-data rejection, artifact generation and command-change logging. Build and all 55 offline tests pass. This is an odometry tracking comparison, not independent ground-truth accuracy validation. No driver, camera stream or robot motion was launched.
