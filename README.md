## DDS Package

This package uses the data distribution service (DDS) to facilitate communications between heterogenous agents without relying on ROS. Below is a summary of all the scripts, along with an important note on communicating locations among agents.

### Communicating Locations
Because this platform allows heterogeneous agents (different robots, sensors, etc.), the internal map representation for each agent may be slightly different. For example, two robots of different height will have a different LIDAR based map, and possibly different admissible locations it can possible access. We need some way to reconcile these different maps. 

To handle these differences in maps, we will designate a single map frame as the reference map. In our case, we use the map of the first agent to join the network as the reference map frame. All communications of location data across the DDS network must then be sent only in this map frame. Agents with a local map different from the reference map therefore must:
- Transform their location data to the reference map before transmitting data, and
- Transform any received location data back to their local map before processing the data

To accomplish this transform, we use a simple linear transformation between frames based on known common points. Each agent must store the location of at least 3 common points in their local map frame. Upon entering the DDS network, they will receive the location of these common points in the reference map frame. Then, the agent can compute the transformation matrix necessary for transforming between the map frames.

The known points should be stored in the `known_points.txt` file.

TODO: Provide details of computing the transformation matrix

### Summary of Scripts
`agent_entry_exit.py`: This script facilitates the entry and exit of agents to the DDS network. 

`communication_manager.py`:

`goal_reader.py`:

`heartbeat.py`:

`location.py`: