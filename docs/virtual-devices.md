# Virtual devices

The `virtual_devices` package (`ros2-vehicle/virtual_devices/`) fakes Linux device files so the real driver nodes can run when the hardware is not connected. They are test tools for this repository, not a simulator (simulation lives in [autonomous-systems-simulation](https://github.com/TecnicoFuelCell/autonomous-systems-simulation)).

## Virtual camera

`virtual_camera_node` streams an image or video file to a [v4l2loopback](https://github.com/v4l2loopback/v4l2loopback) device at `camera_device_path`, in the capture mode configured for `webcam_publisher` in `vehicle_params.yaml`. Then `webcam_node` and everything downstream run unchanged.

### Create and remove the device

The node does not create the device: loading a kernel module needs root on the host, which the container does not have. Create it on the **host** (the Jetson, outside Docker) with the real camera unplugged, so `/dev/video0` is free:

```bash
# One-time install (builds the module for the running kernel; needs its headers)
sudo apt install v4l2loopback-dkms v4l2-utils

# Create /dev/video0 (video_nr must match camera_device_path in vehicle_params.yaml)
sudo modprobe v4l2loopback video_nr=0 card_label="TFC virtual camera" exclusive_caps=1

# Check it exists and is a loopback device
v4l2-ctl --list-devices

# Remove it when done (fails with "Module is in use" while any node still has it open)
sudo modprobe -r v4l2loopback
```

Notes:

- Ubuntu 24.04 ships v4l2loopback 0.12.x, where devices can only be created and removed by loading/unloading the module as above. Version 0.13 and newer also provide `v4l2loopback-ctl add` / `delete` to manage single devices without reloading (see `v4l2loopback-ctl --help`).
- `exclusive_caps=1` makes the device appear as a normal camera (capture-only) once a writer is attached, which is why the virtual camera must start before `webcam_node`.
- Create the device **before** starting the container and pass it in like the real camera (e.g. `--device /dev/video0`); device files created later do not appear inside a running container unless `/dev` is bind-mounted.
- The container needs `gstreamer1.0-tools`, `gstreamer1.0-plugins-base` and `gstreamer1.0-plugins-good`.

### Run

Start the virtual camera first, then launch the camera nodes as usual (`vehicle_launch.xml`, calibration launches, ...):

```bash
ros2 launch virtual_devices virtual_camera.launch.xml input:=<image-or-video>
# optional: playback_args:="--no-loop --stretch"
```

Or without the launch file:

```bash
ros2 run virtual_devices virtual_camera_node <image-or-video> \
    --ros-args --params-file <path to vehicle_params.yaml>
```

Playback options: `--loop` (default) / `--no-loop`, `--frames N`, `--timeout SECONDS`, `--background B,G,R`, `--stretch`.
