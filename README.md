# CNS Flight Stack: Data Recorder ROS1 Package (data_recorder)

[![License](https://img.shields.io/badge/License-AAUCNS-336B81.svg)](./LICENSE) [![Paper](https://img.shields.io/badge/IEEEXplore-10.1109/LRA.2022.3196117-00629B.svg?logo=ieee)](https://doi.org/10.1109/LRA.2022.3196117) [![Release](https://img.shields.io/github/v/release/aau-cns/data_recorder?include_prereleases&logo=github)](https://github.com/aau-cns/data_recorder/releases)

Maintainer: [Martin Scheiber](mailto:martin.scheiber@aau.at)

A ROS wrapper to communicate via ROS services to start execution of data recording scripts and data storing scripts.

## Credit
This code was written by the [Control of Networked System (CNS)](https://www.aau.at/en/smart-systems-technologies/control-of-networked-systems/), University of Klagenfurt, Klagenfurt, Austria.

## License
This software is made available to the public to use (_source-available_), licensed under the terms of the BSD-2-Clause-License with no commercial use allowed, the full terms of which are made available in the `LICENSE` file. No license in patents is granted.

### Usage for academic purposes
If you use this software in an academic research setting, please cite the
corresponding paper and consult the `LICENSE` file for a detailed explanation.

```latex
@article{cns_flightstack22,
    title        = {CNS Flight Stack for Reproducible, Customizable, and Fully Autonomous Applications},
    author       = {Scheiber, Martin and Fornasier, Alessandro and Jung, Roland and Böhm, Christoph and Dhakate, Rohit and Stewart, Christian and Steinbrener, Jan and Weiss, Stephan and Brommer, Christian},
    journal      = {IEEE Robotics and Automation Letters},
    volume       = {7},
    number       = {4},
    year         = {2022},
    doi          = {10.1109/LRA.2022.3196117},
    url          = {https://ieeexplore.ieee.org/document/9849131},
    pages        = {11283--11290}
}
```

## Getting Started

### Prerequesites
This package is part of the [CNS Flight Stack] and thus depends on the other packages of the flight stack:
- [CNS Flight Stack: Autonomy Engine]
- [CNS Flight Stack: Nodelet Rosbag]

Further the following libraries are required
- Eigen
- ROS noetic

### Build

As this is a ROS package, please build it within the catkin environment with

```bash
catkin build data_recorder
```

## Usage
The intended usage is together with the [CNS Flight Stack: Autonomy Engine], which will interact with the data recorder. Use the provided launchfile to start the ROS1 node

```bash
roslaunch data_recorder recorder.launch
```

### Recording scripts
The main goal of this node is to execute several scripts, i) to start data recording, ii) to stop data recording, and iii) to store the recorded data.

- i) is executed when a recording start request is received and is most likely to call the [CNS Flight Stack: Nodelet Rosbag], the arguments given in `record_cmd` are passed to this script
- ii) is executed when a recording stop is received, typically killing all nodes starting with `/record_`
- iii) is executed after (ii) and can be used to move the recorded data (assuming several storage locations) to a final (external) device; the arguments given in `store_cmd` are passed to this script

For a sample configuration of these scripts please take a look at the scripts provided in [CNS Flight Stack Scripts].

### ROS Parameters
This node uses the following parameters that can be set in any launch file.

| ROS parameter | description | default value |
|:-------------:|:-----------:|:-------------:|
| `record_script` | path to the record script which is executed to start the recording       | ` `     |
| `record_cmd`    | arguments passed to the record script                                    | ` `     |
| `record_stop`   | path to the record stop script which is executed to stop the recording   | ` `     |
| `store_script`  | path to the record script which is executed to store the data afterwards | ` `     |
| `store_cmd`     | arguments passed to the store script                                     | ` `     |
| `verbose`       | flag to display debug output                                             | `False` |

### Default Launchfile Parameters

These are the same as the ROS parameters, so only their default value is given.

| Launch parameter | default value                  |
|:----------------:|:------------------------------:|
| `record_script` | `<path_to_data_recorder>/scripts/record.sh` (_dummy script_) |
| `record_cmd`    | `dev1_full` |
| `record_stop`   | `<path_to_data_recorder>/scripts/record_stop.sh` (_generic script to stop all nodes starting with `/record_`_) |
| `store_script`  | `<path_to_data_recorder>/scripts/store.sh` (_dummy script_) |
| `store_cmd`     | ` ` |
| `verbose`       | `false` |

### Usage without Autonomy Engine

If required the detector can be used without the [CNS FlightStack: Autonomy Engine]. You can interact with the detector using the provided ROS service ROS Service `/data_recorder/record` using the
[`std_srvs/SetBool`](http://docs.ros.org/en/api/std_srvs/html/srv/SetBool.html).
The request is interpreted as follows

- `data=true`: start recording (or do nothing if already recording)
- `data=false`: stop recording (or do nothing if already stopped)

The response is `success` is set to `true` if the operation was successful (or mode was already selected).
If any failure occured (script not found, process could not be started, ...) the response is set to `false`.
Further, the `message` filed contains a minor debug information on the recording status.

An example execution looks like

```bash
rosservice call /data_recorder/record "data: false"
# Feedback:
success: true|false
message: "START: ..."|"STOP: ..."
```

## Architecture

Please refer to the academic paper for further insights of the Data Recorder.

## Package Layout

```console
/path/to/data_recorder$ tree -L 3 --noreport --charset unicode
.
|-- CMakeLists.txt
|-- launch
|   `-- recorder.launch
|-- LICENSE
|-- nodes
|   `-- DataRecorderNode.py
|-- package.xml
|-- README.md
|-- scripts
|   |-- record.sh
|   |-- stop_record.sh
|   `-- store.sh
|-- setup.py
`-- src
    `-- data_recorder
        |-- data_recorder.py
        `-- __init__.py
```

---


Copyright (C) 2021-2022 Martin Scheiber and Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
You can contact the authors at [martin.scheiber@aau.at](mailto:martin.scheiber@aau.at?subject=toland_flight%20package), [christian.brommer@aau.at](mailto:christian.brommer@aau.at?subject=data_recorder%20package).


<!-- LINKS: -->
[CNS Flight Stack]: https://github.com/aau-cns/flight_stack
[CNS Flight Stack Scripts]: https://github.com/aau-cns/flight_stack/tree/main/src/flightstack/flightstack_scripts/record_scripts
[CNS Flight Stack: Autonomy Engine]: https://github.com/aau-cns/autonomy_engine
[CNS Flight Stack: Nodelet Rosbag]: https://github.com/aau-cns/nodelet_rosbag
