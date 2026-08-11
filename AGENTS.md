# AGENTS.md

## Repository identity

- This repository is `/home/kublab/ros2_ws/src/go2w_om6dof_agent` on the Jetson AGX.
- The remote login is `kublab@100.86.67.110` over Tailscale.
- Read `application/application_web_monitor/utility/CONNECTING_HP_CHATGPT_TO_AGX.md` before diagnosing SSH, ChatGPT remote, Codex installation, PATH, or authorization problems.
- Do not recreate SSH keys, modify `authorized_keys`, or route through MSI when `ssh agx` from HP already succeeds.

## Working authority and safety

- Codex runs as Linux user `kublab`; repository files are owned by `kublab:kublab` and are already readable and writable.
- Work inside this Git repository unless the user explicitly requests a system-level change.
- `AGENTS.md` supplies instructions; it does not bypass OS permissions, the Codex sandbox, or approval prompts.
- Never use `chmod 777` to solve access problems.
- Ask before using `sudo`, editing `/etc`, installing packages, changing systemd services, changing Tailscale/SSH configuration, or writing outside the repository.
- Treat robot motion, torque, arm ownership, service restarts, and hardware-stack changes as physical-world actions. Require explicit user approval immediately before performing them.
- Prefer read-only checks first. Do not start the robot, move joints, disable torque, or restart hardware during ordinary documentation, code inspection, or tests.
- Preserve unrelated working-tree changes. Do not delete or commit generated `__pycache__`, build, install, or log artifacts unless explicitly requested.
- Never expose private-key contents, access tokens, passwords, or authentication files in output or commits.

## Verified project workflow

- ROS distribution: Humble.
- Workspace root: `/home/kublab/ros2_ws`.
- Package: `application_web_monitor`.
- Build from the workspace root:

```bash
source /opt/ros/humble/setup.bash
cd /home/kublab/ros2_ws
colcon build --symlink-install --packages-select application_web_monitor
source install/setup.bash
unset CYCLONEDDS_URI
```

- Run focused tests:

```bash
cd /home/kublab/ros2_ws/src/go2w_om6dof_agent/application/application_web_monitor
python3 -m pytest -q
```

- Do not assume the Unitree/NX link is available. Standalone AGX mode should keep `go2w_enabled:=false` and `robot_ssh_host` empty unless the user explicitly requests the combined topology.
- The web monitor controls real hardware and has no login. Do not expose port `8080` to the public internet; use the trusted LAN or Tailscale.

## Documentation discipline

- Update `application/application_web_monitor/utility/CONNECTING_HP_CHATGPT_TO_AGX.md` whenever the host alias, Tailscale IP, user, key filename, Codex path, or connection procedure changes.
- Record verified commands and their expected output. Clearly label unverified assumptions.
- Prefer fixing the documented root cause over repeating key generation, installation, or broad permission changes.
