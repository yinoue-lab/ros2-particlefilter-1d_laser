# ベースイメージ
FROM ubuntu:22.04

# 環境変数の設定
ENV DEBIAN_FRONTEND=noninteractive

# 必要なパッケージをインストール
RUN apt-get update && apt-get install -y \
    curl \
    sudo \
    software-properties-common \
    python3 \
    python3-pip \
    ansible \
    && rm -rf /var/lib/apt/lists/*

# GitHub CLI のインストール
RUN type -p curl >/dev/null || sudo apt install curl -y && \
    curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg && \
    sudo chmod go+r /usr/share/keyrings/githubcli-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null && \
    sudo apt update && \
    sudo apt install -y gh ansible

# vcstool のインストール
RUN curl -s https://packagecloud.io/install/repositories/dirk-thomas/vcstool/script.deb.sh | sudo bash && \
    sudo apt-get update && \
    sudo apt-get install -y python3-vcstool

# Ansible Playbook のコピーと実行
COPY ansible /root/ansible
RUN ansible-playbook /root/ansible/dev.yml --connection=local

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

RUN mkdir -p /root/ros2_ws
WORKDIR /root/ros2_ws
RUN git clone https://github.com/yinoue-lab/ros2-particlefilter-1d_laser.git

WORKDIR /root/ros2_ws/ros2-particlefilter-1d_laser
RUN git submodule update --init --recursive

RUN vcs import --recursive < ext.repos
RUN vcs pull ext

RUN ansible-playbook --ask-become-pass ext/blackbox/ansible/docker.yml 

RUN apt update
RUN rosdep update
RUN rosdep install -yi --rosdistro humble --from-paths .

# デフォルトのエントリーポイント
CMD ["/bin/bash"]
