# mattbot_dds

This package uses the [Data Distribution Service (DDS)](https://www.omg.org/spec/DDS/) (CycloneDDS) so heterogeneous agents can exchange data without going through ROS. ROS is still used **on each robot** to bridge DDS to local nodes (navigation, perception, etc.).

**Prerequisite:** set `ROBOT_ID` to a non-empty integer before launching any node in this package. DDS topic names and routing depend on it (see `dds_utils.network.require_robot_id_int`). On each robot you can persist that with a line such as `export ROBOT_ID=2` in `~/.bashrc`, then `source ~/.bashrc` or open a new shell before `roslaunch`.

---

## Directed vs peer data (read this first)

Fleet traffic uses per-agent DDS topics named `DataTopic{agent_id}` (`dds_utils.data_topic_name`).

| Path | DDS reader | Typical `sending_agent` | Purpose |
|------|------------|-------------------------|---------|
| **Directed to this robot** | `own_data_subscriber.py` on **your** `DataTopic{ROBOT_ID}` | Orchestrator / another robot (not self) | Goals, multi-robot goals, position init, unknown-image requests, **human stop** (`MSG_STOP`), **full stack shutdown** (`MSG_ROBOT_SHUTDOWN`) |
| **Peer telemetry** | `data_subscriber.py` on **each peer’s** `DataTopic{peer_id}` (from `/agents_to_subscribe`) | Must equal that **peer’s** id | Objects, paths, map updates, face encodings, STAR tensors, relayed `global_observe_start`, etc. |

Orchestrators (e.g. a central `goal_publisher` that consumes GraphQL) write `DataMessage` samples **onto the target robot’s** `DataTopic{rid}`. Those samples are **not** visible to `data_subscriber.py`’s peer-only pattern; they are handled by **`own_data_subscriber.py`**.

**Orchestrator / human node codebase:** [github.com/satomm1/dds_robot_platform](https://github.com/satomm1/dds_robot_platform) — reference implementation for fleet-facing services (e.g. goals and human stop requests emitted as DDS `DataMessage`s to each robot’s data topic).

---

## Human / fleet stop → ROS

When a `DataMessage` with `message_type == "stop"` arrives on this robot’s data topic (JSON payload often includes `"source": "human"`), **`own_data_subscriber.py`** publishes `std_msgs/Bool` **`data: true`** on ROS (default topic **`/stop`**).

- **Publisher param:** `~stop_ros_topic` (default `/stop`) on node `dds_own_data_subscriber`.
- **Downstream:** e.g. `mattbot_navigation` `localize_and_navigate.py` subscribes to `~/stop_topic` (default `/stop`) and transitions to **IDLE** with zero `cmd_vel` when stopping. Keep these topic names aligned in launch files if you override them.

Constant: `MSG_STOP` in `src/dds_utils/messages.py` (re-exported from `dds_utils`).

---

## Fleet shutdown → end `roslaunch` (graceful stack teardown)

When a `DataMessage` with `message_type == "robot_shutdown"` (`MSG_ROBOT_SHUTDOWN`) arrives on this robot’s `DataTopic{ROBOT_ID}`, **`own_data_subscriber.py`** calls `rospy.signal_shutdown(...)`, the node process exits, and because **`own_data_subscriber` is launched with `required="true"`** in [`launch/dds.launch`](launch/dds.launch), `roslaunch` tears down the other nodes in that session (e.g. `mattbot_bringup` `short.launch` / `tall.launch` which include `dds.launch`).

- **Wire format:** same `DataMessage` as other directed traffic; `data` is JSON (often `{}` or `{"reason": "..."}`).
- **Param:** `~allow_dds_roslaunch_shutdown` on `dds_own_data_subscriber` (default **true**). Set to **false** on a dev machine if you must receive samples without ending the launch.
- **Security:** any writer that can publish to this robot’s `DataTopic{ROBOT_ID}` can trigger a full shutdown when the param is true—same trust model as fleet stop (`MSG_STOP`).
- **Fleet / orchestrator:** publish to **`DataTopic{target_robot_id}`** with `message_type` `"robot_shutdown"`; see [dds_robot_platform](https://github.com/satomm1/dds_robot_platform) for the reference fleet stack.

This is separate from **`MSG_STOP`**, which only publishes **`/stop`** and does not exit the process.

---

## Communicating locations across maps

Because agents may use different local maps (resolution, origin, sensor coverage), the stack assumes a **reference map**. All **shared** poses sent over DDS must be expressed in that reference frame; each agent transforms **outbound** data to the reference frame and **inbound** data back to its local map using a 2D linear map from correspondence points.

> [!IMPORTANT]
> Store correspondence points in [`scripts/known_points.txt`](scripts/known_points.txt), one `x,y` per line (local map). To capture reference-frame coordinates, use the `get_reference_points` launch file from `mattbot_bringup`, drive the robot to the same physical locations, and record them.

Agents must subscribe to the shared ROS transform (`transformation_matrix` / `/transformation_matrix` per `dds_utils.topics`) so bridge nodes can apply `TransformMixin.transform_point`.

---

## Package layout

| Path | Role |
|------|------|
| [`src/dds_utils/`](src/dds_utils/) | Shared constants (`MSG_*`, ROS topic names), `DataMessage` IDL types, QoS, participant helpers, `TransformMixin` |
| [`msg/`](msg/) | Custom ROS messages (`AgentPath`, `MapUpdate`, `MultiRobotGoalPlan`, …) used by bridges |
| [`scripts/`](scripts/) | ROS nodes (Python) |
| [`launch/dds.launch`](launch/dds.launch) | Default stack for a single agent |

---

## Scripts (nodes)

| Script | Node (typical) | Role |
|--------|----------------|------|
| `entry_exit.py` | `agent_entry_exit` | Agent join/leave on the DDS initialization / entry-exit topics |
| `heartbeat_publisher.py` | `heartbeat_publisher` | Publish this agent’s heartbeat |
| `heartbeat_subscriber.py` | `heartbeat_subscriber` | Track other agents’ heartbeats → ROS (`/heartbeat_agents`, etc.) |
| `location_publisher.py` | `location_publisher` | Publish this robot’s pose to DDS |
| `location_subscriber.py` | `location_subscriber` | Other agents’ poses → ROS |
| `data_publisher.py` | `dds_data_publisher` | ROS → DDS: objects, paths, goals, invalid goals, faces, multi-robot plans, optional STAR tensors, optional `global_observe_start` forwarding, air quality (`MSG_AIR_QUALITY` from `/air_quality`, default every 10 s) |
| `data_subscriber.py` | `dds_data_subscriber` | DDS (peers) → ROS: e.g. `/object_from_agent`, `/object_from_sensor`, `/path_from_agent`, `/map_update`, `/new_face_encoding`, `/team/dds/...`, latched global observe relay |
| `own_data_subscriber.py` | `dds_own_data_subscriber` | DDS (**this** `DataTopic`) → ROS: `/external_goal`, `/external_goal_multi`, `/initialpose`, `/send_unknown_images`, **`/stop`**; optional **`robot_shutdown`** → `rospy.signal_shutdown` (required node) |
| `image_publisher.py` | `image_publisher` (when `publish_images:=true`) | JPEG camera frames → DDS `ImageTopic{ROBOT_ID}` (`encoding="jpeg"`); params `~image_topic`, `~publish_rate_hz` (default 2), `~jpeg_quality` (80), `~max_width` (640), `~tall` (180° rotate when camera is upside-down) |

Deprecated and test helpers live under `scripts/deprecated/` and `scripts/testing/`.

---

## `dds.launch` arguments

| Arg | Default | Meaning |
|-----|---------|---------|
| `sqlite` | `false` | Enable SQLite logging in `data_publisher`, `data_subscriber`, `own_data_subscriber` |
| `forward_global_observe_start_via_dds` | `true` | `data_publisher`: ROS trigger → DDS `MSG_GLOBAL_OBSERVE_START` |
| `relay_global_observe_start_from_dds` | `true` | `data_subscriber`: DDS → ROS latched wall time |
| `global_observe_start_ros_topic` | `/global_observe_start` | Output topic for relay |
| `global_observe_start_dds_trigger_topic` | `/global_observe_start_dds` | Input topic for forwarder (avoids echoing `/global_observe_start` back onto DDS) |
| `publish_images` | `true` | Start `image_publisher` (JPEG → `ImageTopic{ROBOT_ID}`) |
| `image_topic` | `/camera/color/image_raw` | ROS image source for DDS publish |
| `image_publish_rate_hz` | `2.0` | Max DDS image publish rate |
| `image_jpeg_quality` | `80` | OpenCV JPEG quality (0–100) |
| `image_max_width` | `640` | Downscale before JPEG if wider; `0` = no resize |
| `tall` | `false` | Pass to `image_publisher`: 180° rotate before JPEG (upside-down tall mount) |

On `data_publisher`, private param `~air_quality_publish_period_s` (default **10.0**) sets how often `/air_quality` is forwarded to DDS as `MSG_AIR_QUALITY` (`"air_quality"`). JSON `data`: `temperature` (°F), `relative_humidity` (%), `voc_index`, `nox_index`. Consumers read `DataTopic{agent_id}` where `sending_agent == agent_id`.

Run:

```bash
roslaunch mattbot_dds dds.launch
```

---

## ROS topics worth remembering

- **`/agents_to_subscribe`** (`Int16MultiArray`): who `data_subscriber` should attach DDS readers to.
- **`transformation_matrix`** / **`/transformation_matrix`**: map-frame transform for bridges.
- **`/stop`** (`Bool`): asserted `true` on human/fleet stop via `own_data_subscriber`; navigation should use the same name or set matching private params.

See `src/dds_utils/topics.py` and each script’s `rospy.get_param` calls for the full list of tunables (`~external_goal_multi_ros_topic`, `~multi_robot_goal_plan_topic`, `~relay_global_observe_start_from_dds`, STAR topic names, sqlite, etc.).

---

**Author:** Matthew Sato, Stanford Engineering Informatics Group

**License:** [MIT License](./LICENSE)
