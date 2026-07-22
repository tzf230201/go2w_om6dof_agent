# application_web_monitor

ROS 2 application dashboard and voice-agent pipeline for the Unitree Go2W and
OM6DOF stack. The application server runs on the Jetson AGX and provides robot
status, forwarded camera/audio, speech-to-text, local LLM actions, teleoperation
controls, and guarded service operations.

The source used by the AGX is:

```text
/home/kublab/ros2_ws/src/go2w_om6dof_agent/application/application_web_monitor
```

Open `http://<agx-ip>:8080` from the trusted robot LAN. The HTTP server binds to
all interfaces and currently has no login, so it must not be exposed directly
to an untrusted network.

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

## Build and test

```bash
cd /home/kublab/ros2_ws
source /home/kublab/unitree_ros2/setup.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select application_web_monitor
source install/setup.bash

cd src/go2w_om6dof_agent/application/application_web_monitor
python3 -m pytest -q
```

Run the dashboard manually with:

```bash
ros2 run application_web_monitor web_monitor \
  --ros-args -p robot_ssh_host:=unitree@192.168.123.18
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
buttons control the user unit `om6dof-perception.service` on `robot_ssh_host`,
and **Set target** publishes `/om6dof_perception/set_target`. Install the user
service file shipped by `om6dof_perception` on the robot/NX first.

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

Both source-specific unit templates are kept in `systemd/`. The installed
generic service name is `application-audio-bridge.service` so downstream STT
dependencies do not change.

Use DJI:

```bash
install -m 0644 systemd/application-dji-audio-user.service \
  ~/.config/systemd/user/application-audio-bridge.service
systemctl --user daemon-reload
systemctl --user restart application-audio-bridge.service application-stt.service
```

Use the Go2W built-in microphone:

```bash
install -m 0644 systemd/application-audio-bridge-user.service \
  ~/.config/systemd/user/application-audio-bridge.service
systemctl --user daemon-reload
systemctl --user restart application-audio-bridge.service application-stt.service
```

The `systemd/` directory contains the source service units. The `sudoers/`
directory contains the narrowly scoped permission used by the guarded OM6DOF
restart button.
