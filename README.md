# Particle Filter with 1D Laser

## Install
```
type -p curl >/dev/null || sudo apt install curl -y
curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg \
&& sudo chmod go+r /usr/share/keyrings/githubcli-archive-keyring.gpg \
&& echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null \
&& sudo apt update \
&& sudo apt install gh ansible -y
```
```
curl -s https://packagecloud.io/install/repositories/dirk-thomas/vcstool/script.deb.sh | sudo bash
sudo apt-get update
sudo apt-get install python3-vcstool
```
```
ansible-playbook --ask-become-pass ansible/dev.yml
```
```
./update.bash
```

## Docker
Install extensions of "Dev Container" to vscode.

```
docker build -t ros2-humble-image docker

docker run -dit --name ros2-container ros2-humble-image
docker exec -it ros2-container /bin/bash
mkdir git
git clone https://github.com/yinoue-lab/ros2-particlefilter-1d_laser.git git && cd git/ros2-particlefilter-1d_laser
```
