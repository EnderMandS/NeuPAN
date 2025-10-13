FROM ubuntu:focal

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ARG USERNAME=ubuntu
ARG TARGETPLATFORM

ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
ENV XAUTHORITY=/tmp/.docker.xauth

RUN apt update && \
    apt install -y tree wget curl git unzip zip && \
    apt install -y build-essential cmake && \
    if [ "$TARGETPLATFORM" = "linux/amd64" ]; then apt install -y vim; fi && \
    apt install -y zsh && \
    apt install -y libeigen3-dev && \
    DEBIAN_FRONTEND=noninteractive apt install -y keyboard-configuration && \
    apt install -y x11-apps mesa-utils && \
    apt install -y sudo && \
    rm -rf /var/lib/apt/lists/*

# setup user
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID ${USERNAME} \
    && useradd --uid $USER_UID --gid $USER_GID -m ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}
USER ${USERNAME}

# zsh
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended
RUN git clone --depth 1 https://github.com/zsh-users/zsh-autosuggestions /home/${USERNAME}/.oh-my-zsh/custom/plugins/zsh-autosuggestions && \
    git clone --depth 1 https://github.com/zsh-users/zsh-syntax-highlighting.git /home/${USERNAME}/.oh-my-zsh/custom/plugins/zsh-syntax-highlighting && \
    sed -i 's/plugins=(git)/plugins=(git zsh-autosuggestions zsh-syntax-highlighting)/g' /home/${USERNAME}/.zshrc
SHELL ["/bin/zsh", "-c"]

WORKDIR /home/${USERNAME}/code

# Install miniconda and mamba
RUN wget -c https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh && \
    bash Miniconda3-latest-Linux-x86_64.sh -b -u -p /home/${USERNAME}/miniconda3 && \
    rm Miniconda3-latest-Linux-x86_64.sh && \
    /home/${USERNAME}/miniconda3/bin/conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    /home/${USERNAME}/miniconda3/bin/conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r && \
    /home/${USERNAME}/miniconda3/bin/conda init zsh && \
    /home/${USERNAME}/miniconda3/bin/conda install -n base -c conda-forge mamba -y

# Install ROS 2 Humble
RUN /home/${USERNAME}/miniconda3/bin/mamba create -n neupan && \
    /home/${USERNAME}/miniconda3/bin/conda config --env --add channels conda-forge && \
    /home/${USERNAME}/miniconda3/bin/conda config --env --remove channels defaults && \
    /home/${USERNAME}/miniconda3/bin/conda config --env --add channels robostack-${ROS_DISTRO} && \
    /home/${USERNAME}/miniconda3/bin/mamba install -n neupan ros-${ROS_DISTRO}-desktop -y && \
    /home/${USERNAME}/miniconda3/bin/mamba install -n neupan -c conda-forge colcon-common-extensions catkin_tools rosdep

RUN git clone --depth 1 --recursive https://github.com/EnderMandS/NeuPAN && \
    cd NeuPAN && \
    /home/${USERNAME}/miniconda3/bin/mamba run -n neupan pip install -e .
    
RUN echo 'eval "$(~/miniconda3/bin/mamba shell hook --shell zsh)"' >> /home/${USERNAME}/.zshrc && \
    echo "mamba activate neupan" >> /home/${USERNAME}/.zshrc && \
    echo ": 1700000000:0;roscore" >> /home/$USERNAME/.zsh_history && \
    echo ": 1700000001:0;python neupan/ros/neupan_ros.py" >> /home/$USERNAME/.zsh_history

ENTRYPOINT [ "/bin/zsh" ]