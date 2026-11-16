# RoboMaster raw TCP video queue fix

The installed DJI Python SDK `StreamConnection._recv_task` reads 4096-byte chunks into a 32-entry queue. On full queue it removes one old chunk and fails to enqueue the new chunk. Losing arbitrary bytes from the H.264 TCP stream corrupts the decoder input; recovery can wait for a keyframe. The existing ROS camera decoder also drains until empty with repeated byte-string concatenation, and its decoded-image queue keeps an older frame instead of replacing it with the newest one when full.

The patch adds an optional receiver inside `robomaster_ros`, without editing the installed SDK. It replaces the original video receiver before streaming starts; it uses the same robot camera endpoint and a single TCP socket. A bounded queue applies TCP backpressure and never discards active encoded bytes. Decoder batches are bounded to eight chunks and joined once. Only decoded output frames may be replaced. Image stamps use the final contributing chunk's ROS receive time, rather than a fresh stamp after dequeue. These are receipt timestamps, not camera exposure timestamps.

The shared driver's default `camera.video.low_latency` is false, preserving the Phase 2 default behavior. Phase 3 passes `video_low_latency:=true` and `video_stats_period_s:=5.0` to the existing driver launch. These new launch arguments are backed by the patched driver's parameters. No estimator, steering, arrival or chassis code is changed by this patch. Existing user changes in the driver repository were preserved.

On a fresh checkout of the matching driver, apply and rebuild:

```bash
git -C ~/ros2_ws/src/robomaster_ros apply --check ~/ros2_ws/src/rm_phase3_odometry/driver_patch/low_latency_video.patch
git -C ~/ros2_ws/src/robomaster_ros apply ~/ros2_ws/src/rm_phase3_odometry/driver_patch/low_latency_video.patch
cd ~/ros2_ws
colcon build --packages-select robomaster_ros rm_phase3_odometry --symlink-install
source install/setup.bash
```

The patch is already applied in this workspace. Do not reapply it. A running Python driver retains its already-created receiver; rebuilding does not hot-replace that receiver. The fix takes effect on the next driver start. No running process was stopped or restarted while implementing it. If using an independently launched driver, enable the new driver launch arguments there; `start_driver:=false` cannot change an existing connection's receiver.

Every five seconds `[Camera pipeline]` reports:

- `tcp_kBps`, `receive_idle_ms`, `max_receive_gap_ms`, `socket_timeouts`: whether the video socket delivers bytes regularly. Backpressure can also increase receive gaps; inspect queue depth/waits together.
- `decoded_fps`, `max_decode_ms`, `decode_errors`: decoder progress and failures.
- `published_fps`, `max_publish_ms`: publisher progress and DDS blocking time. Successful publish calls do not prove subscriber receipt.
- `byte_queue`, `backpressure_waits`, `max_queue_ms`: compressed-byte backlog and decoder lag.
- `decoded_replaced`: safely replaced decoded frames, not discarded H.264 fragments.
- `tcp_error`: EOF/socket failure, if present.

Tests verify full byte continuity with a deliberately full queue, bounded batch draining, end-of-stream handling, newest-decoded-frame behavior and shutdown with a full queue. A fake-frame integration check against the actual driver decoder verifies timestamp propagation and output replacement. No network/robot connection was created during these checks. This fixes demonstrated software defects; the actual robot must still establish the resulting FPS. Wi-Fi outages, robot encoder stalls, SDK codec defects or DDS delivery losses cannot be ruled out from offline tests.

To roll back this feature without modifying the driver source, pass `video_low_latency:=false` on the next Phase 3 launch. The source patch can also be reversed with `git apply --reverse`, provided no subsequent overlapping edits were made.
