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
    click n3 "https://github.com/TecnicoFuelCell/autonomous-systems-vehicle"
    click n4 "https://github.com/TecnicoFuelCell/autonomous-systems-simulation"

```

- **Autonomous core logic:** [autonomous-systems-core](https://github.com/TecnicoFuelCell/autonomous-systems-core)
- **Vehicle setup (this repo):** [autonomous-systems-vehicle](https://github.com/TecnicoFuelCell/autonomous-systems-vehicle)
- **Simulation setup:** [autonomous-systems-simulation](https://github.com/TecnicoFuelCell/autonomous-systems-simulation)
- **Data I/O interface:** planned repository for the shared ROS 2 topics and messages