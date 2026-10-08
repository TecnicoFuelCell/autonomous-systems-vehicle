# Vehicle setup

This repository is the **real-car I/O layer** Autonomous Systems stack. It connects the prototype’s sensors and actuators to the autonomy software: data comes in from the vehicle, commands go out to the traction motor and steering motor.

## In the autonomy stack

The autonomy stack is split so the same core logic can run on the car or in simulation. A dedicated **data I/O interface** repository will hold the shared ROS 2 contract. This repository **implements that contract on the physical vehicle**. The simulation repository does the same against virtual sensors and actuators.

```mermaid
---
config:
  theme: mc
---
flowchart TB
    n1["Autonomous core logic"] <--> n2["Data I/O interface"]
    n2 <--> n3["Vehicle setup"] & n4["Simulation setup"]

    n1@{ shape: proc}
    n2@{ shape: rect}
    n3@{ shape: rect}
    n4@{ shape: rect}
    style n3 stroke-width:4px,stroke-dasharray: 0,fill:#C8E6C9
    click n1 "https://github.com/TecnicoFuelCell/autonomous-systems-core"
    click n2 "https://github.com/TecnicoFuelCell/autonomous-systems-io"
    click n3 "https://github.com/TecnicoFuelCell/autonomous-systems-vehicle"
    click n4 "https://github.com/TecnicoFuelCell/autonomous-systems-simulation"

```

- **Autonomous core logic:** [autonomous-systems-core](https://github.com/TecnicoFuelCell/autonomous-systems-core)
- **Data I/O interface:** [autonomous-systems-io](https://github.com/TecnicoFuelCell/autonomous-systems-io)
- **Vehicle setup (this repo):** [autonomous-systems-vehicle](https://github.com/TecnicoFuelCell/autonomous-systems-vehicle)
- **Simulation setup:** [autonomous-systems-simulation](https://github.com/TecnicoFuelCell/autonomous-systems-simulation)

## Project structure

### Upstream data
```mermaid
---
config:
  layout: elk
---
flowchart BT
 subgraph s1["ROS2 vehicle"]
        n1["deadman_pub"]
        n2["dir_pub"]
        n3["gps_pub"]
        n4["imu_pub"]
        n5["joystick_pub"]
        n6["mag_pub"]
        n7["uart_reader"]
        n8["vesc_pub"]
        n9["webcam_pub"]
        n12["joy_node"]
        n19["Data I/O interface"]
  end
 subgraph s2["Linux files"]
        n10["/dev/ttyUSB1"]
        n11["/dev/input/js0"]
        n13["/dev/ttyACM0"]
        n14["/dev/video0"]
  end
 subgraph s3["Physical devices"]
        n15["GPS module"]
        n16["PS4 controller"]
        n17["CANToPC"]
        n18["ElGato webcam"]
  end
    n7 -- /vehicle_internal/serial/acc <br>/vehicle_internal/serial/gyro --> n4
    n7 -- /vehicle_internal/serial/dir --> n2
    n7 -- /vehicle_internal/serial/deadman --> n1
    n7 -- /vehicle_internal/serial/vesc --> n8
    n7 -- /vehicle_internal/serial/mag --> n6
    n11 --> n12
    n13 --> n7
    n12 -- /joy --> n5
    n14 --> n9
    n10 --> n3
    n15 -- USB --> n10
    n16 -- bluetooth --> n11
    n17 -- USB --> n13
    n18 -- USB --> n14
    n3 --> n19
    n5 --> n19
    n9 --> n19
    n6 --> n19
    n8 --> n19
    n1 --> n19
    n2 --> n19
    n4 --> n19

    n2@{ shape: rect}
    n3@{ shape: rect}
    n4@{ shape: rect}
    n5@{ shape: rect}
    n6@{ shape: rect}
    n7@{ shape: rect}
    n8@{ shape: rect}
    n9@{ shape: rect}
    n19@{ shape: rect}
    n10@{ shape: rect}
    n11@{ shape: rect}
    n13@{ shape: rect}
    n15@{ shape: rect}
    n16@{ shape: rect}
    n17@{ shape: rect}
    n18@{ shape: rect}
    style n12 fill:#BDBCCC,stroke-width:1px,stroke-dasharray: 1
```
- **joy_node:** belongs to ROS2 library, not implementation of this project