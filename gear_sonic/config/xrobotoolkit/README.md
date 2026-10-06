# XRoboToolkit MUJOCO Video Profile

## What this configures

This directory provides the optional `MUJOCO` entry for XRoboToolkit Remote
Vision's installed `video_source.yml`. The entry configures a 1280x480,
side-by-side H.264 video source for the MuJoCo ego camera.

## Requirements

- A PICO headset running a version of XRoboToolkit Remote Vision that supports `video_source.yml`.
- USB debugging and ADB access to the headset for profile installation.

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

Check the names and parse the merged YAML locally before pushing it. This is
only a profile syntax check. It uses PyYAML from `.venv_sim`:

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

## Use the installed profile

For MuJoCo startup, bridge commands, network ports, expected logs, and runtime
troubleshooting, see
[View the MuJoCo Ego Camera in XRoboToolkit](../../../docs/source/tutorials/data_collection.md#view-the-mujoco-ego-camera-in-xrobotoolkit).

## Restore the original profile

```bash
adb push ./video_source.yml.backup /sdcard/Android/data/com.xrobotoolkit.client/files/video_source.yml
adb shell am force-stop com.xrobotoolkit.client
```

Restart XRoboToolkit on the headset and confirm its original video sources return.

## Profile troubleshooting

- **`adb devices` shows no device:** Check the USB connection and USB mode,
  enable developer options and USB debugging, accept the headset authorization
  prompt, then retry `adb kill-server && adb start-server` and `adb devices`.
- **`MUJOCO` is missing:** Parse and inspect the merged YAML, confirm the push
  went to `/sdcard/Android/data/com.xrobotoolkit.client/files/video_source.yml`,
  and restart the headset application.
