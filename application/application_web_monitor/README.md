# application_web_monitor

ROS 2 application dashboard for OM6DOF, with optional Unitree Go2W integration.
The application server runs on the Jetson AGX and provides robot
status, forwarded camera/audio, speech-to-text, local LLM actions, teleoperation
controls, and guarded service operations.

The source used by the AGX is:

```text
/home/kublab/ros2_ws/src/go2w_om6dof_agent/application/application_web_monitor
```

Open `http://<agx-ip>:8080` from the trusted robot LAN. The HTTP server binds to
all interfaces and currently has no login, so it must not be exposed directly
to an untrusted network.

## Top status ribbon

The sticky header keeps the most useful live values visible while the page is
scrolled:

- the browser's local time in `HH:MM` format;
- RAM usage as a memory-chip gauge and percentage;
- robot battery state of charge as a battery gauge and percentage.

Hover over the RAM or battery indicator to see the detailed value. RAM details
show used and total memory, while battery details show voltage and current. The
header values are refreshed from `/status.json` once per second without a page
reload. RAM is green below 70%, amber from 70% through 84%, and red at 85% or
higher. Battery is green at 40% or higher, amber from 20% through 39%, and red
below 20%.

## Current pipeline

```text
DJI Wireless Mic Rx
  -> PulseAudio stereo 48 kHz
  -> mono PCM + SpeexDSP noise suppression
  -> VAD utterance segmentation
  -> Whisper large-v3 on CUDA
  -> final STT event
  -> Qwen3-VL Robot Agent through Ollama
  -> bounded ROS action + response in the web monitor
  -> Kokoro natural TTS -> 44.1 kHz WAV -> Go2W audiohub speaker
```

The web microphone playback toggle only controls playback and network streaming
to that browser. Disabling playback does not stop audio capture, STT, or the
Robot Agent.

## Desktop shortcut and monitor mode

The complete desktop-shortcut implementation is stored in [`utility/`](utility):

- `select_web_monitor_mode.sh` shows the OM6DOF / Go2W / combined mode picker,
  saves the choice, restarts only the web-monitor service, and opens the local
  dashboard in the browser.
- `launch_web_monitor.sh` is the service entry point. It loads the Unitree ROS
  environment only for Go2W-containing modes.
- `OM6DOF-Web-Monitor.desktop` is the GNOME desktop shortcut.
- `om6dof-web-monitor-launcher.sudoers` grants the shortcut permission to
  restart only `om6dof-web-monitor.service`.

To install or recreate the shortcut on the AGX after cloning or updating this
repository:

```bash
REPO=/home/kublab/ros2_ws/src/go2w_om6dof_agent/application/application_web_monitor

chmod 0755 "$REPO/utility/launch_web_monitor.sh" \
  "$REPO/utility/select_web_monitor_mode.sh"
install -m 0755 "$REPO/utility/OM6DOF-Web-Monitor.desktop" \
  "$HOME/Desktop/OM6DOF-Web-Monitor.desktop"
gio set "$HOME/Desktop/OM6DOF-Web-Monitor.desktop" metadata::trusted true

sudo install -m 0644 "$REPO/systemd/om6dof-web-monitor.service" \
  /etc/systemd/system/om6dof-web-monitor.service
sudo install -m 0440 "$REPO/utility/om6dof-web-monitor-launcher.sudoers" \
  /etc/sudoers.d/om6dof-web-monitor-launcher
sudo visudo -cf /etc/sudoers.d/om6dof-web-monitor-launcher
sudo systemctl daemon-reload
```

Then double-click **Web Monitor Robot** on the AGX desktop and choose one mode:
**OM6DOF only**, **Go2W only**, or **Go2W + OM6DOF**. The selected mode is
saved in `~/.config/om6dof-web-monitor/mode.env`; the current default is
OM6DOF. Go2W telemetry and control still require the Go2W network link to be
present.

### MoveIt desktop shortcut

`utility/run_moveit.sh` and `utility/OM6DOF-MoveIt.desktop` provide a separate
MoveIt launcher. Before opening RViz, it reports:

- whether `om6dof-hardware.service` is active;
- the reported Dynamixel torque state from
  `/dynamixel_hardware_interface/dxl_state`;
- whether measured `/joint_states` is inside MoveIt's `home_pose` rest area
  (each arm joint within ±0.25 rad).

The dialog offers three actions: use the current hardware stack, restart the
stack before starting MoveIt, or stop the stack and open MoveIt in planning-only
mode. The planning-only option starts MoveIt with `start_hardware:=false`; it
must not be used for real robot execution.

Install or recreate it with:

```bash
REPO=/home/kublab/ros2_ws/src/go2w_om6dof_agent/application/application_web_monitor

chmod 0755 "$REPO/utility/run_moveit.sh" "$REPO/utility/moveit_rest_check.py"
install -m 0755 "$REPO/utility/OM6DOF-MoveIt.desktop" \
  "$HOME/Desktop/OM6DOF-MoveIt.desktop"
gio set "$HOME/Desktop/OM6DOF-MoveIt.desktop" metadata::trusted true

sudo install -m 0440 "$REPO/utility/om6dof-moveit-launcher.sudoers" \
  /etc/sudoers.d/om6dof-moveit-launcher
sudo visudo -cf /etc/sudoers.d/om6dof-moveit-launcher
```

Use the **OM6DOF MoveIt** icon from the AGX desktop. Review the reported rest
area and torque state before selecting a destructive stack action. The shortcut
never launches a second OM6DOF hardware instance; MoveIt always uses
`start_hardware:=false`.

## Build and test

Install the runtime dependency used to inspect ros2_control:

```bash
sudo apt update
sudo apt install ros-humble-controller-manager-msgs
```

For the standalone AGX mode, the Unitree environment is not required:

```bash
cd /home/kublab/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select application_web_monitor
source install/setup.bash
unset CYCLONEDDS_URI

cd src/go2w_om6dof_agent/application/application_web_monitor
python3 -m pytest -q
```

Run the dashboard manually with:

```bash
ros2 run application_web_monitor web_monitor \
  --ros-args -p go2w_enabled:=false
```

`unset CYCLONEDDS_URI` is important when the shell previously sourced the
Unitree setup, which pins CycloneDDS to `eno1`. Without the NX or Ethernet
cable, that interface may be inactive and ROS will fail to create a node.
Standalone mode lets DDS select an available AGX interface.

Open `http://<agx-ip>:8080`. In this mode the header explicitly shows
**Go2W disabled**. Go2W camera, battery, microphone, F3 teleop, TTS speaker, and
locomotion Robot Agent controls are not created or displayed. OM6DOF hardware
status/restart, arm ownership/modes, absolute arm targets, RealSense
perception, and DD-GNG remain available locally on the AGX. Losing the network
connection to the Jetson NX therefore does not stop the dashboard.

Do not set `robot_ssh_host` in this topology: an empty value makes all OM6DOF
systemd checks and controls local to the AGX.

## Live OM6DOF position visualization

The dashboard includes a dependency-free 3D kinematic view next to the web
joystick. It subscribes directly to measured `sensor_msgs/msg/JointState`
feedback on `/joint_states`; it does not animate from commanded targets. The
browser reads `/joint_state.json` at 10 Hz and smoothly renders joints 1–6,
the end effector, and gripper opening. Drag the view to rotate it and use the
mouse wheel or trackpad to zoom.

The green `LIVE` badge includes the age of the latest feedback. It changes to
`STALE` after one second without a new message, which helps distinguish a
stopped Dynamixel/controller pipeline from a browser rendering issue. Verify
the data source independently with:

```bash
ros2 topic hz /joint_states
ros2 topic echo --once /joint_states
curl -fsS http://127.0.0.1:8080/joint_state.json
```

The viewer is embedded in the monitor and does not require rosbridge, Three.js,
a CDN, or internet access.

## Install as an AGX system service

After building, install the supplied standalone unit and the narrowly scoped
sudo rule used by the OM6DOF restart button:

```bash
sudo install -o root -g root -m 0644 \
  /home/kublab/ros2_ws/install/application_web_monitor/share/application_web_monitor/systemd/om6dof-web-monitor.service \
  /etc/systemd/system/om6dof-web-monitor.service
sudo install -o root -g root -m 0440 \
  /home/kublab/ros2_ws/install/application_web_monitor/share/application_web_monitor/sudoers/om6dof-web-monitor \
  /etc/sudoers.d/om6dof-web-monitor
sudo visudo -cf /etc/sudoers.d/om6dof-web-monitor
sudo systemctl daemon-reload
sudo systemctl enable --now om6dof-web-monitor.service
```

The sudoers rule is intentionally assigned to AGX user `kublab`. If the file
still contains the former `unitree` user, the dashboard restart button will
fail with `sudo: a password is required`; reinstall the file after rebuilding.

Verify it with:

```bash
systemctl status om6dof-web-monitor.service
journalctl -u om6dof-web-monitor.service -n 100 --no-pager
curl -fsS http://127.0.0.1:8080/status.json
```

To restore the combined dashboard later, run with `go2w_enabled:=true`, source
the Unitree environment, and set `robot_ssh_host` only if OM6DOF itself has
again been moved to the remote host.

## Accessing the web monitor from another computer

The dashboard listens on port `8080`. A device on the same LAN can open:

```text
http://<agx-lan-ip>:8080
```

For access from a different Wi-Fi, mobile network, or another location, install
Tailscale on the AGX and on the client computer or phone, then sign in to the
same Tailnet. Do not expose port `8080` directly to the public internet: the
dashboard can operate the arm and should remain private.

On the AGX, verify the connection and obtain its Tailscale IP:

```bash
tailscale status
tailscale ip -4
```

Then open the dashboard from any authorized Tailscale device:

```text
http://<agx-tailscale-ip>:8080
```

If MagicDNS is enabled, the stable hostname can be used instead:

```text
http://<agx-hostname>.<tailnet-name>.ts.net:8080
```

Example for this AGX Tailnet at the time of installation:

```text
http://agx.tail455172.ts.net:8080
```

The monitor service binds to `0.0.0.0:8080`, so no extra firewall or router
port-forwarding rule is required for Tailscale access. Confirm the monitor is
running before troubleshooting the client connection:

```bash
systemctl is-active om6dof-web-monitor.service
```

### Phone-first arm control

Open the dedicated mobile page instead of the desktop dashboard:

```text
http://<agx-tailscale-ip>:8080/mobile
```

For this AGX, the current MagicDNS address is:

```text
http://agx.tail455172.ts.net:8080/mobile
```

The page is designed as a game-style control screen: the RealSense camera fills
the screen, two semi-transparent touch joysticks overlay the video, and a
compact live TF/joint-state mini-map remains in the upper-left corner. The
left stick controls axis pair 1 (axes 1/2) and the right stick controls axis
pair 2 (axes 3/4). Both pairs are combined into one bounded six-axis ROS
command, so one stick cannot overwrite the other. The mode selector changes
the meanings of those axes between JOINT, CARTESIAN, and CYLINDRICAL.

Enable control, Stop control, Rest, and camera controls remain available as
compact overlays. The page uses the same ROS control path and safeguards as the
desktop page.

The touch joystick is deliberately **hold-to-move**: releasing the finger,
leaving the page, losing browser focus, or losing the network sends a zero
velocity command. Enable control first, select the intended mode and axis pair,
and keep the arm workspace clear before moving. Use **Stop control** to return
to autonomous ownership; **Disable torque** is guarded because the arm can
fall under gravity.

## Resolved issue: Perception start failed from another computer

**Symptom:** Selecting **Start perception** in the web monitor showed a
failure such as:

```text
Failed to connect to bus: $DBUS_SESSION_BUS_ADDRESS and $XDG_RUNTIME_DIR not defined
```

**Cause:** `om6dof-web-monitor.service` is a system service. It runs as
`kublab`, but it does not inherit the desktop session variables required by a
plain `systemctl --user` command. The failure was independent of the browser,
the client computer, and Tailscale.

**Fix:** The web monitor service explicitly connects to the persistent
`kublab` user manager by setting these service environment variables:

```bash
XDG_RUNTIME_DIR=/run/user/1000
DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
```

It then invokes the user unit through the same user manager. This lets the
Start/Stop Perception and DD-GNG controls work from any browser that can reach
the monitor. User lingering must remain enabled so the user manager is
available without an interactive desktop login:

```bash
loginctl show-user kublab -p Linger
# Expected: Linger=yes
```

## Camera forwarding

The camera card is hidden until forwarded JPEG frames arrive on:

```text
/application_web_monitor/image/compressed
```

The message type is `sensor_msgs/msg/CompressedImage`. The card disappears
automatically when frames stop. The AGX camera relay obtains the Go2W built-in
front camera through Unitree's `videohub` API:

```bash
ros2 run application_web_monitor unitree_camera_relay
```

The two camera paths are independent:

```text
Go2W built-in camera: /application_web_monitor/image/compressed
RealSense processing: /application_web_monitor/perception/image/compressed
```

Start perception to show the second, processed preview:

```bash
ros2 launch om6dof_perception perception.launch.py
```

The dashboard also provides an **OM6DOF perception** card. Its Start/Stop
buttons control the local AGX user unit `om6dof-perception.service`, and
**Set target** publishes `/om6dof_perception/set_target`. No SSH call to the NX
is involved.

Install all optional OM6DOF application services locally for user `kublab`:

```bash
mkdir -p ~/.config/systemd/user
install -m 0644 \
  ~/ros2_ws/install/om6dof_perception/share/om6dof_perception/systemd/om6dof-perception-user.service \
  ~/.config/systemd/user/om6dof-perception.service
install -m 0644 \
  ~/ros2_ws/install/om6dof_pick_and_place/share/om6dof_pick_and_place/systemd/om6dof-perception-pick-user.service \
  ~/.config/systemd/user/om6dof-perception-pick.service
install -m 0644 \
  ~/ros2_ws/install/om6dof_dd_gng/share/om6dof_dd_gng/systemd/om6dof-dd-gng.service \
  ~/.config/systemd/user/om6dof-dd-gng.service
systemctl --user daemon-reload
```

Perception/pick and DD-GNG conflict because both open the same RealSense. The
GUI starts and stops these local units as mutually exclusive workloads.

## DJI microphone and audio filter

The active input is the DJI USB receiver:

```text
Pulse source: auto-selected source whose name contains `DJI`
USB device:   DJI Technology Co., Ltd. Wireless Mic Rx
Input:        stereo 48 kHz
ROS output:   mono 48 kHz signed 16-bit little-endian PCM
```

Capture uses PulseAudio rather than opening ALSA directly. This avoids
`Device or resource busy` after boot when PulseAudio already owns the USB
capture device. If the receiver is absent or capture ends, systemd retries the
bridge automatically. The bridge also clears the PulseAudio source mute state
at startup; a muted source otherwise produces valid-looking frames containing
only digital silence.

`dji_audio_bridge` applies SpeexDSP noise suppression at `-18 dB`. The filtered
audio is used both for live web playback and STT. VAD only determines utterance
boundaries; the GUI displays `VOICE DETECTED` or `NO VOICE DETECTED`.

Current timing values:

- pre-roll: 300 ms, preserving the beginning of a word;
- end-of-speech hangover: 1200 ms, allowing short pauses inside a sentence;
- audio frame: 20 ms / 960 samples;
- normal publish rate: 50 frames per second.

The original Go2W microphone decoder remains available as a fallback:

```bash
ros2 run application_web_monitor unitree_audio_bridge
```

It subscribes to `/audiosender`, decodes Opus, and publishes through the same
application audio topics.

## Speech-to-text

The STT server is whisper.cpp with CUDA acceleration on the AGX:

```text
Model:    /mnt/agx_nvme/whisper.cpp/models/ggml-large-v3.bin
Language: English (`en`)
Decoder:  beam search, size 5
Server:   http://127.0.0.1:8178
```

STT is utterance based, not partial-token streaming. Processing begins after
VAD marks the end of speech. The final transcript is latched for the web page,
while a separate volatile event prevents an old transcript from being executed
again after a service restart.

## Voice Robot Agent

Final STT events are sent to the local Ollama model:

```text
qwen3-vl:8b-instruct-q4_K_M
```

The model converts the utterance to one action using
`skills/go2w_control_skill.md`. The web monitor validates and executes the JSON
action through the Unitree sport API.

Motion commands require a wake word at the beginning:

- `Robot, move forward one meter.`
- `Robots, turn left ninety degrees.`
- `Hey robot, stand up.`
- `Hey robots, what is the battery level?`

`Stop` is intentionally accepted without a wake word. Other transcripts without
`Robot`, `Robots`, `Hey robot`, or `Hey robots` are ignored. This gate prevents
background speech and Whisper hallucinations from becoming robot actions.

## Natural voice and half-duplex audio

Robot Agent responses are synthesized on the AGX with Kokoro ONNX. The active
voice is `af_heart` (US English), with model data stored on the NVMe:

```text
/mnt/agx_nvme/kokoro/kokoro-v1.0.onnx
/mnt/agx_nvme/kokoro/voices-v1.0.bin
```

The TTS node converts Kokoro's output to mono 44.1 kHz WAV and uploads it to
the Go2W built-in body speaker using audiohub megaphone requests on
`/api/audiohub/request` (enter `4001`, data `4003`, exit `4002`). The AGX only
runs inference and sends the audio; it does not play through the AGX sound card.
Only the volatile `/application/llm/event` is spoken, so restarting the TTS
service does not repeat an old answer.

Audio is intentionally half duplex for now. Before speaker playback starts,
`/application/tts/speaking` becomes `true`; the DJI bridge then publishes
digital silence to the optional web-audio stream and sends nothing to STT.
After playback it waits another 700 ms before reopening the microphone. This
prevents the robot from transcribing its own voice and starting a feedback loop.
The web monitor displays the TTS state while this gate is active.

An in-memory 32-entry LRU cache makes repeated responses skip synthesis. The
experimental per-sentence streaming mode is disabled by default because the
current Go2W audiohub firmware inserts audible gaps between separate WAV files;
one continuous WAV is used for smooth speech.

Supported actions include bounded move/turn/strafe, stop, stand, lie down,
recover, gait, speed, damp, teleop, battery, status, topic listing, and node
listing. Movement is capped by the executor:

```text
vx:       -0.4 .. 0.4 m/s
vy:       -0.3 .. 0.3 m/s
wz:       -0.8 .. 0.8 rad/s
duration: 0 .. 8 seconds, followed by StopMove
```

Voice actions can physically move the robot. Keep the operating area clear and
use `Stop` to cancel normal motion. `Robot, emergency stop` invokes motor damp
and may cause the robot body to sag.

### Camera questions with the VLM

Explicit visual questions take a thread-safe snapshot of the latest JPEG on
`/application_web_monitor/image/compressed` and send it to the local Qwen3-VL
model through Ollama `/api/chat`. They bypass the motion-action parser, so a
camera description cannot accidentally become a movement command. Examples:

- `Robot, what do you see?`
- `Robot, describe the camera view.`
- `Robot, is there a person in front of you?`
- `Robot, how many people are there?`

The answer is shown in the web monitor and spoken through the same Kokoro and
Go2W speaker path. Visual answers are limited to one short sentence (at most
25 words) to minimize VLM-to-speech latency. A recent camera frame is required; otherwise the agent
returns a camera-unavailable error instead of analyzing a stale image.

## ROS topics

| Topic | Type | Purpose |
|---|---|---|
| `/application/audio/pcm_s16le` | `std_msgs/msg/UInt8MultiArray` | Filtered continuous mono PCM for the web |
| `/application/audio/speech_s16le` | `std_msgs/msg/UInt8MultiArray` | Filtered frames inside the active utterance |
| `/application/audio/voice_active` | `std_msgs/msg/Bool` | Current VAD state |
| `/application/audio/format` | `std_msgs/msg/String` | Latched JSON source/format metadata |
| `/application/stt/text` | `std_msgs/msg/String` | Latest final transcript, transient-local |
| `/application/stt/event` | `std_msgs/msg/String` | New final transcript event, volatile |
| `/application/stt/status` | `std_msgs/msg/String` | STT state |
| `/application/llm/response` | `std_msgs/msg/String` | Latest Robot Agent response |
| `/application/llm/event` | `std_msgs/msg/String` | New response event consumed once by TTS |
| `/application/llm/status` | `std_msgs/msg/String` | Agent state, including wake-word waiting |
| `/application/tts/status` | `std_msgs/msg/String` | TTS loading, synthesis, playback, or ready state |
| `/application/tts/speaking` | `std_msgs/msg/Bool` | Latched half-duplex microphone gate |
| `/api/audiohub/request` | `unitree_api/msg/Request` | WAV upload to the Go2W body speaker |
| `/application_web_monitor/image/compressed` | `sensor_msgs/msg/CompressedImage` | Forwarded camera or perception overlay |
| `/api/sport/request` | `unitree_api/msg/Request` | Validated Go2W sport action output |

## Auto-run services

The AGX user services are enabled at boot:

| Service | Function |
|---|---|
| `ollama.service` | Local Qwen LLM server |
| `whisper-stt-server.service` | CUDA Whisper `large-v3` HTTP server |
| `application-audio-bridge.service` | Active DJI capture/filter bridge |
| `application-stt.service` | Speech segmentation and transcription client |
| `application-stt-llm.service` | Wake-word gate and Robot Agent bridge |
| `application-tts.service` | Kokoro synthesis and Go2W speaker bridge |
| `application-web-camera-relay.service` | Unitree front-camera forwarding |
| `om6dof-web-monitor.service` | ROS monitor and HTTP dashboard |

Check the complete pipeline with:

```bash
systemctl --user is-enabled \
  ollama.service whisper-stt-server.service \
  application-audio-bridge.service application-stt.service \
  application-stt-llm.service application-tts.service \
  application-web-camera-relay.service \
  om6dof-web-monitor.service

systemctl --user is-active \
  ollama.service whisper-stt-server.service \
  application-audio-bridge.service application-stt.service \
  application-stt-llm.service application-tts.service \
  application-web-camera-relay.service \
  om6dof-web-monitor.service
```

`loginctl show-user kublab -p Linger` must report `Linger=yes` so the user
services start without an interactive login. The NVMe is mounted persistently at
`/mnt/agx_nvme` through `/etc/fstab`.

View recent service output with:

```bash
systemctl --user status application-audio-bridge.service
systemctl --user status whisper-stt-server.service
systemctl --user status application-stt.service
systemctl --user status application-stt-llm.service
systemctl --user status application-tts.service
systemctl --user status om6dof-web-monitor.service
```

## Selecting the microphone service

The dashboard audio card discovers PulseAudio microphones and speakers on the
AGX. Go2W microphone and speaker choices appear only while the `eno1` Ethernet
link has carrier. Select both devices and press **Enable audio** to start the
microphone, STT, LLM, and TTS services. **Disable audio** stops them again.

The audio services remain disabled at login and have a systemd start limit of
three failures per minute. A missing device, disconnected Go2W, or missing
model therefore produces a bounded failure instead of an endless restart loop.
The separate **Enable live audio** button controls monitoring playback only in
the current browser.

The `systemd/` directory contains the source service units. The `sudoers/`
directory contains the narrowly scoped permission used by the guarded OM6DOF
restart button.

## RealSense low-light mode

The dashboard has a **RealSense low-light mode** card. Enable it before starting
Perception or DD-GNG to request the connected camera's low-light setting. On a
camera with an IR emitter, it enables the emitter and applies the configured
laser power. On the installed D405, which exposes no controllable IR emitter or
laser-power option, it enables auto exposure instead. If a camera workload is
already active, the dashboard restarts only that workload so the setting takes
effect; it does not touch the arm stack.

The shared setting is stored on the AGX at:

```text
~/.config/om6dof-realsense/low_light.json
```

The default laser power is `150`. It is safely clamped to the range reported by
a device that supports it. Low-light mode does not make YOLO RGB labels see in
the dark:
YOLOX still needs visible light to identify object names. Use a small white LED
when semantic labels are needed in darkness.
