# Remote access and Codex on the AGX

This file records the connection state verified from the HP Windows laptop so
future operators and AI agents do not repeat trial-and-error setup.

## Device topology

| Device | Role | SSH endpoint | Key on HP |
| --- | --- | --- | --- |
| HP | ChatGPT/Codex client | local Windows PC | n/a |
| MSI | Lab PC | `teuku@100.93.232.22:22` | `C:/Users/HP/.ssh/msi_codex` |
| AGX | Research edge device and ROS 2 host | `kublab@100.86.67.110:22` | `C:/Users/HP/.ssh/hp_agx` |

All listed addresses are Tailscale addresses. Normal use is HP directly to
AGX; MSI is not required for routine AGX access.

## Known-good HP configuration

`C:/Users/HP/.ssh/config`:

```sshconfig
Host agx
    HostName 100.86.67.110
    User kublab
    Port 22
    IdentityFile C:/Users/HP/.ssh/hp_agx
    IdentitiesOnly yes
```

Verification:

```powershell
ssh -o BatchMode=yes -o ConnectTimeout=10 agx hostname
```

Expected output: `agx`.

The dedicated HP public-key fingerprint installed in
`/home/kublab/.ssh/authorized_keys` is:

```text
SHA256:4U2zOKNOuX8dQwKCmo8rs1w8oQN7tqS/36fpfXsNj8U HP-to-AGX (ED25519)
```

Never copy the private key into this repository or display its contents.

## How the working key was installed

The HP key pair is `C:/Users/HP/.ssh/hp_agx` and `hp_agx.pub`. The public key
was sent to AGX through the already-authorized MSI-to-AGX path. The final
direct test used only `hp_agx` and succeeded.

Do not repeat key installation unless the direct verification command fails
and the fingerprint is absent from `authorized_keys`. Repeated appends can
create duplicate or encoding-corrupted lines.

## Codex CLI required by ChatGPT remote projects

Codex was installed with the official Linux standalone installer. Verified on
2026-08-13:

```text
/home/kublab/.local/bin/codex
codex-cli 0.147.0
Logged in using ChatGPT
```

The installer appended PATH below the standard `.bashrc` early return, so
ChatGPT's non-interactive SSH command could not see it. The working fix is at
the beginning of `/home/kublab/.bashrc`:

```bash
# Codex SSH PATH
export PATH="$HOME/.local/bin:$PATH"
```

Verify all prerequisites through the same SSH mode:

```powershell
ssh agx "command -v codex; codex --version; codex app-server --help >/dev/null; codex login status"
```

## Troubleshooting order

1. Run `ssh agx hostname` from HP.
2. If authentication fails, inspect `ssh -G agx`; confirm `kublab`, the
   absolute `hp_agx` identity path, and `IdentitiesOnly yes`.
3. If terminal SSH works but ChatGPT does not, fully quit ChatGPT, reopen it,
   and remove/re-add the cached `agx` connection.
4. If ChatGPT reports Codex missing, run the non-interactive verification.
   Fix PATH before reinstalling.
5. Do not use `chmod 777`. The repository is owned by `kublab:kublab` and is
   already writable by the account used by Codex.

## Repository and authority scope

The correct Git root is:

```text
/home/kublab/ros2_ws/src/go2w_om6dof_agent
```

This tutorial is intentionally stored in the package `utility/` directory.
`AGENTS.md` remains at the Git root because Codex automatically reads guidance from the root down to
the working directory. Its authority section defines safe operating boundaries
but does not grant Linux privileges or bypass ChatGPT approvals.

Ordinary authorized work includes reading repository files, editing requested
source or documentation, and running documented build/tests. System files,
package installation, systemd changes, networking changes, and commands that
can move or de-energize real robot hardware require explicit user approval.

## Project commands

```bash
cd /home/kublab/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select application_web_monitor
source install/setup.bash
unset CYCLONEDDS_URI

cd src/go2w_om6dof_agent/application/application_web_monitor
python3 -m pytest -q
```

See `application/application_web_monitor/README.md` for systemd, dashboard,
ROS, MoveIt, and physical-safety details.
