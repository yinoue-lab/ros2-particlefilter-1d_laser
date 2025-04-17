# Particle Filter with 1D Laser

## Install

### Install Github CLI and Ansible
```sh
type -p curl >/dev/null || sudo apt install curl -y
curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg \
&& sudo chmod go+r /usr/share/keyrings/githubcli-archive-keyring.gpg \
&& echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null \
&& sudo apt update \
&& sudo apt install gh ansible -y
```

### Install vcs-tool
```sh
curl -s https://packagecloud.io/install/repositories/dirk-thomas/vcstool/script.deb.sh | sudo bash
sudo apt-get update
sudo apt-get install python3-vcstool
```

### Install ros2
```sh
git clone --recursive https://github.com/yinoue-lab/ros2-particlefilter-1d_laser.git
ansible-playbook --ask-become-pass ansible/dev.yml
```

### Install dependency packages for this ros2_ws
```sh
./update.bash

```

## Docker
```sh
git clone --recursive https://github.com/yinoue-lab/ros2-particlefilter-1d_laser.git
docker build -t ros2-humble-image .
```

For rviz
```sh
xhost +local:
```

```sh
docker run \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -dit --name ros2-container ros2-humble-image
```

Install the Dev Container extension in VSCode. Then, using the VSCode Dev Container feature, attach to the ros2-container and open `/root/ros2_ws/ros2-particlefilter-1d_laser`.

In container
```sh
./build.sh
source install/setup.bash
ros2 launch simulation_launcher simulation_headless.launch.py
```