# Environment Setup Guide

## Target Stack

- Windows 10/11
- WSL 2
- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo Harmonic

Notes:

- Do not install `gazebo` or `gazebo11`
- `gazebo` refers to Gazebo Classic, not Gazebo Harmonic

---

## 1. Install WSL

Open PowerShell as Administrator and run:

```powershell
wsl --install
```

Check WSL status:

```powershell
wsl --status
wsl --version
```

Set WSL 2 as default:

```powershell
wsl --set-default-version 2
```

Update WSL if needed:

```powershell
wsl --update
```

---

## 2. Install Ubuntu 24.04

List available distributions:

```powershell
wsl --list --online
```

Install Ubuntu 24.04:

```powershell
wsl --install -d Ubuntu-24.04
```

If the download gets stuck:

```powershell
wsl --install --web-download -d Ubuntu-24.04
```

Launch Ubuntu:

```powershell
wsl -d Ubuntu-24.04
```

On first launch, create your Linux username and password.

Check Ubuntu version:

```bash
lsb_release -a
uname -a
```

---

## 3. Update Ubuntu

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y curl gnupg lsb-release software-properties-common locales
```

---

## 4. Configure Locale

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
```

---

## 5. Install ROS 2 Jazzy

Enable the `universe` repository:

```bash
sudo add-apt-repository universe
sudo apt update
```

Add the ROS 2 apt source:

```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

Install ROS 2 Jazzy:

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools
```

Load ROS automatically:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify ROS:

```bash
printenv ROS_DISTRO
which ros2
ros2 topic -h
```

Expected:

- `ROS_DISTRO` = `jazzy`
- `ros2` exists in `/opt/ros/jazzy/bin/ros2`

---

## 6. Install Gazebo Harmonic

Add the Gazebo apt source:

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
```

Install Gazebo Harmonic:

```bash
sudo apt install -y gz-harmonic
```

Install ROS-Gazebo integration:

```bash
sudo apt install -y ros-jazzy-ros-gz
```

Verify Gazebo:

```bash
which gz
gz sim --version
```

---

## 7. Launch Gazebo

Launch Gazebo directly:

```bash
gz sim
```

Launch an empty world:

```bash
gz sim empty.sdf
```

Launch Gazebo from ROS:

```bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf
```

---

## 8. Environment Variables

Recommended `~/.bashrc` entries:

```bash
source /opt/ros/jazzy/setup.bash
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8
```

Reload:

```bash
source ~/.bashrc
```

---

## 9. Useful Checks

WSL:

```powershell
wsl --status
wsl --version
wsl --list --verbose
```

Ubuntu:

```bash
lsb_release -a
```

ROS:

```bash
printenv ROS_DISTRO
which ros2
ros2 topic list
```

Gazebo:

```bash
which gz
gz sim --version
```

---

## 10. Common Issues

WSL install stuck:

```powershell
wsl --shutdown
wsl --update
wsl --install --web-download -d Ubuntu-24.04
```

`Wsl/InstallDistro/0x80072f78`:

- switch network
- disable proxy or filtering software
- retry with `--web-download`

`ros2 --version` fails:

- this is normal
- use `printenv ROS_DISTRO`, `which ros2`, or `ros2 topic -h`

`gz: command not found`:

- Gazebo Harmonic is not installed correctly
- repeat Section 6

Gazebo GUI does not open in WSL:

- check WSLg
- update WSL
- check graphics driver support

---

## 11. Final Verification

Run:

```bash
printenv ROS_DISTRO
which ros2
which gz
gz sim --version
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf
```

Expected:

- ROS 2 Jazzy is available
- Gazebo Harmonic is available
- Gazebo launches correctly
