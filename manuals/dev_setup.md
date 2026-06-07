# Developer Platform Setup 

This README explains the set up for the new Windows 11 development computer for the CORAL project. Target result:

- One Windows developer workstation.
- Two RockPro64 targets:
  - upstairs / office target
  - downstairs / lab target
- VS Code connects to either board through Remote SSH.
- GUI applications open on the Windows workstation through X11 forwarding.
- GitLab/GitHub remains the central code repository.
- Each board has a stable network identity and separate SSH host identity.

---

## 1. Naming convention


| Physical location | Linux hostname | VS Code / SSH alias |  IP |
|---|---|---|---|
| Upstairs office | `stc-office` | `STCOffice` | `192.168.1.65` |
| Downstairs lab | `stc-lab` | `STCLab` | `192.168.1.66` |
| GitLab server | `gitlab-stc` | `gitlab-stc` | `192.168.1.242` |

---

## 2. Network design
Use DHCP reservation on each router:

- Office router reserves the office board MAC address to a fixed IP.
- Lab router reserves the lab board MAC address to a fixed IP.

This is safer than hard-coding static IPs on the board because the board can still boot correctly if moved to another router.

### Configure static IP on the RockPro64

On the target board:

```bash
ip a
ip route
```

Find the Ethernet interface name, usually `eth0` or `end0`.

Then edit:

```bash
sudo nano /etc/systemd/network/eth0.network
```

For the office board:

```ini
[Match]
Name=eth0

[Network]
Address=192.168.1.65/24
Gateway=192.168.1.1
DNS=192.168.1.1
DNS=8.8.8.8
```

For the lab board:

```ini
[Match]
Name=eth0

[Network]
Address=192.168.1.66/24
Gateway=192.168.2.1
DNS=192.168.2.1
DNS=8.8.8.8
```

Apply:

```bash
sudo systemctl disable --now dhcpcd.service 2>/dev/null || true
sudo systemctl enable --now systemd-networkd
sudo systemctl restart systemd-networkd
ip a
ip route
```
Put both routers/subnets under one routed LAN/VLAN or add static routes between the routers to be able to reach both boards from the PC.


---

## 3. Set unique hostnames on the two RockPro64 boards

On the office board:

```bash
sudo hostnamectl set-hostname stc-office
sudo nano /etc/hosts
```

Ensure it contains:

```text
127.0.0.1   localhost
127.0.1.1   stc-office
```

On the lab board:

```bash
sudo hostnamectl set-hostname stc-lab
sudo nano /etc/hosts
```

Ensure it contains:

```text
127.0.0.1   localhost
127.0.1.1   stc-lab
```

Reboot each board:

```bash
sudo reboot
```

After reconnecting:

```bash
hostname
hostnamectl
```

---

## 4. Critical step if the two boards were cloned from the same image

If both RockPro64s were created from the same OS image, they may have identical SSH server host keys. That is bad because SSH uses host keys to prove that each target machine is really the machine you think it is. Run this once on **each** board after cloning/reimaging:

```bash
sudo rm -f /etc/ssh/ssh_host_*
sudo ssh-keygen -A
sudo systemctl restart sshd
```

Then, from the Windows PC, remove old cached host entries if needed:

```powershell
ssh-keygen -R STSide.local
ssh-keygen -R stc-office.local
ssh-keygen -R stc-lab.local
ssh-keygen -R 192.168.1.65
ssh-keygen -R 192.168.2.65
```

---

## 5. Programs to install on the Windows developer computer

### Required

1. **Visual Studio Code**
2. **Git for Windows**
3. **OpenSSH Client**
   - Windows 11 already includes it.
4. **VcXsrv Windows X Server**
   - Needed to display Linux GUI windows on Windows through SSH X11 forwarding.


### Recommended VS Code extensions

Install from VS Code or run:

```powershell
code --install-extension ms-vscode-remote.remote-ssh
code --install-extension ms-vscode.cpptools
code --install-extension ms-vscode.cmake-tools
code --install-extension mhutchie.git-graph
code --install-extension yzhang.markdown-all-in-one
```

## 6. Generate SSH keys on the new Windows PC

Open PowerShell:

```powershell
mkdir $env:USERPROFILE\.ssh -Force
```

Generate one SSH key for connecting from the Windows PC to the RockPro64 boards:

```powershell
ssh-keygen -t ed25519 -a 100 -f $env:USERPROFILE\.ssh\id_ed25519_stc_targets -C "new-dev-pc-to-stc-targets"
```

Generate a separate key for GitLab/GitHub if needed:

```powershell
ssh-keygen -t ed25519 -a 100 -f $env:USERPROFILE\.ssh\id_ed25519_gitlab_stc -C "new-dev-pc-to-gitlab-stc"
```

Start the Windows SSH agent:

```powershell
Get-Service ssh-agent | Set-Service -StartupType Automatic
Start-Service ssh-agent
ssh-add $env:USERPROFILE\.ssh\id_ed25519_stc_targets
ssh-add $env:USERPROFILE\.ssh\id_ed25519_gitlab_stc
ssh-add -l
```

---

## 7. Add the Windows PC public key to both RockPro64 boards

If password login is still active, run from PowerShell:

```powershell
type $env:USERPROFILE\.ssh\id_ed25519_stc_targets.pub | ssh alma@192.168.1.65 "mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"

type $env:USERPROFILE\.ssh\id_ed25519_stc_targets.pub | ssh alma@192.168.2.65 "mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
```


---

## 8. SSH client config on Windows

Create or edit:

```text
C:\Users\<YOUR_WINDOWS_USER>\.ssh\config
```

Use this template. Replace the IPs and Windows username.

```sshconfig
Host STCOffice
  HostName 192.168.1.65
  User alma
  IdentityFile C:/Users/<YOUR_WINDOWS_USER>/.ssh/id_ed25519_stc_targets
  IdentitiesOnly yes
  HostKeyAlias stc-office-rockpro64
  ForwardX11 yes
  ForwardX11Trusted yes
  ServerAliveInterval 30
  ServerAliveCountMax 3

Host STCLab
  HostName 192.168.1.66
  User alma
  IdentityFile C:/Users/<YOUR_WINDOWS_USER>/.ssh/id_ed25519_stc_targets
  IdentitiesOnly yes
  HostKeyAlias stc-lab-rockpro64
  ForwardX11 yes
  ForwardX11Trusted yes
  ServerAliveInterval 30
  ServerAliveCountMax 3

Host gitlab-stc
  HostName 192.168.1.242
  User git
  IdentityFile C:/Users/<YOUR_WINDOWS_USER>/.ssh/id_ed25519_gitlab_stc
  IdentitiesOnly yes
  HostKeyAlias stc-gitlab
```

Notes:

- `Host STCOffice` and `Host STCLab` are local aliases used by SSH and VS Code.
- `HostKeyAlias` prevents known-host confusion if IPs change or overlap between routers.
- `ForwardX11Trusted yes` is useful for Qt GUI forwarding.

Test:

```powershell
ssh -v STCOffice hostname
ssh -v STCLab hostname
```

Expected outputs:

```text
stc-office
stc-lab
```

---

## 9. Configure SSH server on each RockPro64

On each board:

```bash
sudo systemctl enable --now sshd
sudo nano /etc/ssh/sshd_config
```

Recommended settings:

```text
PubkeyAuthentication yes
X11Forwarding yes
X11UseLocalhost yes
AllowUsers alma
```

Only after key login is tested, optionally disable passwords:

```text
PasswordAuthentication no
```

Restart SSH:

```bash
sudo systemctl restart sshd
```

Do not close your current working SSH session until you have tested a new session successfully.

---

## 10. Install required packages on each RockPro64 target

For Manjaro/Arch ARM:

```bash
sudo pacman -Syu
sudo pacman -S --needed \
  base-devel \
  gcc \
  make \
  cmake \
  ninja \
  git \
  gdb \
  pkgconf \
  inetutils \
  can-utils \
  libgpiod \
  minicom \
  xorg-xauth \
  xorg-xclock \
  xorg-xeyes \
  qt5-base \
  qt5-tools \
  qt6-base \
  qt6-tools \
  can-utils
```

If Qt GUI fails with an `xcb` plugin error, install the XCB helper packages:

```bash
sudo pacman -S --needed \
  libxcb \
  xcb-util \
  xcb-util-image \
  xcb-util-keysyms \
  xcb-util-renderutil \
  xcb-util-wm
```

On some Manjaro/Arch ARM installations, this command may fail:

```bash
sudo pacman -S --needed can-utils
```

with an error like:

```bash
error: target not found: can-utils
warning: 'can-utils' is a file, did you mean -U/--upgrade instead of -S/--sync?
```
 
If can-utils is not available through pacman, build and install it from the official Linux-CAN source repository:

```bash
sudo pacman -S --needed git base-devel cmake make gcc pkgconf

cd /tmp
rm -rf can-utils
git clone https://github.com/linux-can/can-utils.git

cmake -S can-utils -B can-utils/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local

cmake --build can-utils/build -j"$(nproc)"
sudo cmake --install can-utils/build
```

Verify the installation:
```bash
command -v candump
command -v cansend
command -v cangen
candump --help | head
```

Expected output paths are usually:

```bash
/usr/local/bin/candump
/usr/local/bin/cansend
/usr/local/bin/cangen
```

If the commands were installed but are not found by the shell, make sure /usr/local/bin is in the system path:
```bash
echo 'export PATH=/usr/local/bin:$PATH' | sudo tee /etc/profile.d/local-bin.sh
source /etc/profile.d/local-bin.sh
```

Verify again:

```bash
candump --help | head
```
---

## 11. VS Code Remote SSH usage

1. Open VS Code.
2. Press `Ctrl+Shift+P`.
3. Select:

```text
Remote-SSH: Connect to Host...
```

5. Choose either:

```text
STCOffice
STCLab
```

6. Open folder:

```text
/home/alma/STC
```

In the VS Code remote terminal:

```bash
hostname
pwd
```

You should see either `stc-office` or `stc-lab`.

---

## 12. X11 forwarding for GUI applications

### 12.1 Start VcXsrv on Windows

Open XLaunch and use:

```text
Display settings: Multiple windows
Client startup: Start no client
Extra settings: Clipboard enabled
Access control: disable access control if needed for simplicity
```

Allow VcXsrv through Windows Firewall on private networks. For a stricter setup, avoid disabling access control and rely only on SSH X11 forwarding. For initial debugging, disabling access control is often the fastest way to confirm the pipeline.

### 12.2 Test X11 from PowerShell

```powershell
ssh -Y STCOffice
```

On the board:

```bash
echo $DISPLAY
xclock
```

Expected:

- `echo $DISPLAY` shows something like `localhost:10.0`
- `xclock` opens on the Windows desktop

Test the lab board too:

```powershell
ssh -Y STCLab
```

```bash
echo $DISPLAY
xclock
```

### 12.3 Run the STC diagnostic GUI

From the remote shell:

```bash
cd ~/STC
QT_DEBUG_PLUGINS=1 ./build/diag_gui_qt
```

If using VS Code Remote SSH and the integrated terminal does not inherit X11 correctly, use one of these fixes:

1. Start the GUI from a separate PowerShell `ssh -Y STCOffice` session.
2. Install a VS Code Remote X11 helper extension.
3. Confirm server-side `xauth` exists:

```bash
which xauth
```

4. Confirm SSH server config has:

```text
X11Forwarding yes
```

Do not manually set `DISPLAY` unless debugging a specific issue. Normally SSH sets it automatically.

---

## 13. Build STC on a RockPro64

From a VS Code Remote SSH terminal or normal SSH session:

```bash
cd ~/STC
mkdir -p build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)
```

Run server in simulation mode:

```bash
./build/stc_server --tcp-port 7000
```

Run with CAN, example:

```bash
./build/stc_server --can can0 --node 1 --tcp-port 7000
```

Run diagnostic GUI:

```bash
QT_DEBUG_PLUGINS=1 ./build/diag_gui_qt
```

---

## 14. CAN/SocketCAN setup on the RockPro64

Check CAN adapter:

```bash
lsusb
ip link
sudo dmesg | grep -i -E 'can|pcan|peak|usb'
```

For PEAK PCAN-USB, the kernel driver is usually `peak_usb`:

```bash
lsmod | grep peak
```

Bring up `can0` at 1 Mbps:

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

Test logging:

```bash
candump -tz -cae can0
```

Useful filtered log for Nanotec node 1 style traffic:

```bash
candump -tz -cae can0,141:7FF,181:7FF,281:7FF,381:7FF,581:7FF,701:7FF
```

Save a log:

```bash
mkdir -p ~/STC/logs/candump
candump -tz -cae can0 | tee ~/STC/logs/candump/candump_$(date +%F_%H%M%S).log
```

---

## 15. Setup summary
1. Rename boards: `stc-office`, `stc-lab`.
2. Reserve static IPs in routers.
3. Generate new Windows SSH keys:
   - one for board SSH
   - one for GitLab/GitHub
4. Add board SSH public key to both boards.
5. Add Git public key to Git.
6. Use separate SSH aliases: `STCOffice`, `STCLab`, `gitlab-stc`.
7. Install VS Code + Remote SSH + Git + VcXsrv.
8. Install board packages.
9. Test:
   - `ssh STCOffice hostname`
   - `ssh STCLab hostname`
   - `ssh -Y STCOffice xclock`
   - `ssh -T gitlab-stc`
10. Clone/build/run STC on the selected board.
