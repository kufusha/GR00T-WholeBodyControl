# XRoboToolkit MuJoCo Ego View

## What this changes

This adds an optional, standalone display path from the MuJoCo `ego_view` camera to XRoboToolkit Remote Vision on PICO. Recording, the camera viewer, the tmux data-collection launcher, PICO pose input, and episode controls remain unchanged. The bridge only forwards video; it does not replace `pico_manager_thread_server.py`.

## Requirements

- A PICO headset running a version of XRoboToolkit Remote Vision that supports `video_source.yml`.
- USB debugging and ADB access to the headset for profile installation.
- The simulation PC and PICO on the same LAN. Use the PC's LAN IP in Remote Vision.
- The project's `.venv_data_collection` environment with PyAV and a working `libx264` encoder. If the environment is missing, run `bash install_scripts/install_data_collection.sh` from the repository root.

XRoboToolkit compatibility and the full connection path still require validation on the actual headset.

## Back up the installed profile

From the repository root, connect the headset over USB, authorize its debugging prompt, and run:

```bash
adb devices
adb pull /sdcard/Android/data/com.xrobotoolkit.client/files/video_source.yml ./video_source.yml.backup
grep '^- name:' ./video_source.yml.backup
```

Confirm that ADB lists the headset as `device` and that the backup contains the installed `PICO4U` and `ZEDMINI` entries. Keep this backup for recovery.

## Merge the MUJOCO profile

`gear_sonic/config/xrobotoolkit/mujoco_video_source.yml` is a single YAML list item, not a replacement for the installed file. Copy the backup and edit the copy:

```bash
cp ./video_source.yml.backup ./video_source.yml.merged
vi ./video_source.yml.merged
```

Keep every existing entry and field. After the installed `PICO4U` and `ZEDMINI` list items, add the complete `MUJOCO` item from `gear_sonic/config/xrobotoolkit/mujoco_video_source.yml` at the same list indentation (the `- name:` line begins at column 1). Do not add a second top-level YAML document or replace the original items. The added item is:

```yaml
- name: "MUJOCO"
  camera: "ZED"
  description: "MuJoCo ego view from GR00T WholeBodyControl"
  properties:
    - name: "visibleRatio"
      type: "float"
      description: "Visible ratio for the shader"
      value: 0.555
    - name: "contentRatio"
      type: "float"
      description: "Content ratio for the shader"
      value: 1.8
    - name: "heightCompressionFactor"
      type: "float"
      description: "Height compression factor for the shader"
      value: 1.333333
    - name: "RawImageRectSize"
      type: "string"
      description: "Size of the RawImage rectangle for the stereo view"
      value: "600x225"
    - name: "CamWidth"
      type: "int"
      description: "Combined side-by-side frame width"
      value: 1280
    - name: "CamHeight"
      type: "int"
      description: "Combined side-by-side frame height"
      value: 480
    - name: "CamFPS"
      type: "int"
      description: "Video frame rate"
      value: 30
    - name: "CamBitrate"
      type: "int"
      description: "Video bitrate in bits per second"
      value: 1000000
```

Check the names and parse the merged YAML locally before pushing it. The Python command requires PyYAML, declared by the simulation extra and available in `.venv_sim`:

```bash
grep '^- name:' ./video_source.yml.merged
.venv_sim/bin/python -c 'import sys, yaml; data = yaml.safe_load(open(sys.argv[1])); assert isinstance(data, list); names = [item["name"] for item in data]; assert all(names.count(name) == 1 for name in ("PICO4U", "ZEDMINI", "MUJOCO")); assert names.index("MUJOCO") > max(names.index("PICO4U"), names.index("ZEDMINI")); print(names)' ./video_source.yml.merged
```

The printed list must include the original sources exactly once and `MUJOCO` exactly once, after both original entries. Review the complete merged file before installation.

## Install and reload the profile

```bash
adb push ./video_source.yml.merged /sdcard/Android/data/com.xrobotoolkit.client/files/video_source.yml
adb shell am force-stop com.xrobotoolkit.client
```

Restart XRoboToolkit on the headset after the force-stop. Alternatively, close and restart the headset application. Open its video-source selection and verify that `MUJOCO` appears alongside the preserved `PICO4U` and `ZEDMINI` sources.

## Run data collection and the bridge

Terminal 1: start data collection with your existing, user-selected launcher command, unchanged. For the MuJoCo launcher workflow, that command is:

```bash
python gear_sonic/scripts/launch_data_collection.py --sim
```

Terminal 2:

```bash
cd /home/kufushatec/mansub/GR00T-WholeBodyControl
source .venv_data_collection/bin/activate
python gear_sonic/scripts/stream_ego_view_to_pico.py
```

On PICO, select `MUJOCO`, press `Listen`, and enter the simulation PC's LAN IP. The launcher, bridge, and headset listener may be started in any order; streaming begins once all three are ready and the simulation publishes `ego_view`. Stop the bridge with Ctrl-C.

Use `python gear_sonic/scripts/stream_ego_view_to_pico.py --help` to see optional overrides for the camera endpoint, key, control listener, PICO video port, FPS, and bitrate. The defaults match the profile above: 30 FPS and 1000000 bits/s.

## Network contract

| Connection | Direction | Default |
| --- | --- | --- |
| MuJoCo camera ZMQ (`ego_view`) | Bridge subscribes on the PC | `tcp://localhost:5555` |
| Remote Vision control TCP | PICO connects to the bridge on the PC | `0.0.0.0:13579` listener |
| H.264 video TCP | Bridge connects from the PC to PICO | `PICO_IP:12345` |

`0.0.0.0` is a listen address, not the address to enter on PICO; enter the PC's LAN IP. If the PC firewall blocks these connections, allow inbound TCP 13579 from the PICO and outbound TCP 12345 to it. For example, with UFW and a PICO at `192.168.1.50`:

```bash
sudo ufw allow from 192.168.1.50 to any port 13579 proto tcp
sudo ufw allow out to 192.168.1.50 port 12345 proto tcp
```

Use the actual headset IP and your site's firewall policy. These rules are examples; they are only needed if the firewall blocks the traffic. Camera ZMQ stays on the PC with the default `localhost` setting.

## Expected logs

At startup, expect `Control listening on 0.0.0.0:13579` and `Camera endpoint: tcp://localhost:5555 (ego_view)`. After PICO requests video, expect `PICO target: <PICO_IP>` and, once a camera frame is available and the video socket connects, `Video connected to <PICO_IP>:12345`. On Ctrl-C, expect `Stopping XRoboToolkit ego stream`. The bridge does not print a line for each frame.

## Troubleshooting

- **`adb devices` shows no device:** Check the USB connection and USB mode, enable developer options and USB debugging, accept the headset authorization prompt, then retry `adb kill-server && adb start-server` and `adb devices`.
- **`MUJOCO` is missing:** Parse and inspect the merged YAML, confirm the push went to `/sdcard/Android/data/com.xrobotoolkit.client/files/video_source.yml`, and restart the headset application.
- **Blank image:** Confirm the simulation publishes `ego_view` on camera ZMQ `localhost:5555`; check any `--camera-host`, `--camera-port`, or `--camera-key` overrides and confirm that PyAV can open `libx264`.
- **Distorted aspect:** Keep the profile at 1280x480. The bridge duplicates one 640x480 eye image into the 1280x480 side-by-side frame.
- **High latency:** Confirm the bridge's ZMQ subscriber uses `CONFLATE` to keep the latest frame, check LAN quality, lower `--bitrate` if needed, and avoid relaying video through USB networking.
- **Pose works but video does not:** Pose input and Remote Vision use independent connections. Check inbound TCP 13579 to the PC and outbound TCP 12345 to PICO.
- **Video works but controls do not:** This bridge does not replace `pico_manager_thread_server.py`. Check the existing PICO pose and episode-control path separately.

## Restore the original profile

```bash
adb push ./video_source.yml.backup /sdcard/Android/data/com.xrobotoolkit.client/files/video_source.yml
adb shell am force-stop com.xrobotoolkit.client
```

Restart XRoboToolkit on the headset and confirm its original video sources return.

## Current limitations

The bridge duplicates a monoscopic image into both eyes; it does not provide stereo depth or a head-coupled camera. It requires a separate manual launch. XRoboToolkit version compatibility, display geometry, and end-to-end behavior still require hardware validation on PICO.
