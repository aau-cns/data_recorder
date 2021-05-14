# ROS Data Recorder

A ROS wrapper to communicate via ROS services to start execution of data recording scripts.

## License
This software is made available to the public to use (_source-available_), 
licensed under the terms of the BSD-2-Clause-License with no commercial use allowed, 
the full terms of which are made available in the `LICENSE` file. No license in patents is granted.

<!--
### Usage for academic purposes
If you use this software in an academic research setting, please cite the
corresponding paper and consult the `LICENSE` file for a detailed explanation.

```latex
@inproceedings{cite_key,
   author   = {},
   journal  = {},
   title    = {},
   year     = {2021},
}
```
-->

## Usage


### ROS Interface

#### Service Interaction
This node provides the ROS Service `/data_recorder/record` using the 
[`std_srvs/SetBool`](http://docs.ros.org/en/api/std_srvs/html/srv/SetBool.html).
The request is interpreted as follows

- `data=true`: start recording (or do nothing if already recording)
- `data=false`: stop recording (or do nothing if already stopped)

The response is `success` is set to `true` if the operation was successful (or mode was already selected).
If any failure occured (script not found, process could not be started, ...) the response is set to `false`.
Further, the `message` filed contains a minor debug information on the recording status.

#### Parameters
This node uses the following parameters that can be set in any launch file.

| Name            | Description                  |
|:---------------:|:----------------------------:|
| `record_script` | path to the record script    |
| `verbose`       | flag to display debug output |

---

Copyright (C) 2021 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
