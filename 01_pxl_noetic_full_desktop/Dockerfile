FROM osrf/ros:noetic-desktop-full-focal
# The OSRF ROS Noetic containers use the root user.
# Therefore, the following commands are executed as root up until the
# USER user statement.

# We love UTF!
ENV LANG C.UTF-8

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Set the nvidia container runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Install some handy tools.
RUN set -x \
	&& apt-get update \
	&& apt-get --with-new-pkgs upgrade -y \
	&& apt-get install -y mesa-utils \ 
	&& apt-get install -y iputils-ping \ 
	&& apt-get install -y apt-transport-https ca-certificates \
	&& apt-get install -y openssh-server python3-pip exuberant-ctags \
	&& apt-get install -y git vim tmux nano htop sudo curl wget gnupg2 \
	&& apt-get install -y bash-completion \
	&& apt-get install -y libcanberra-gtk3-0 \
	&& apt-get install -y ros-noetic-turtlebot3 \
	&& apt-get install -y ros-noetic-turtlebot3-bringup ros-noetic-turtlebot3-description \
	&& apt-get install -y ros-noetic-turtlebot3-example ros-noetic-turtlebot3-gazebo \
	&& apt-get install -y ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3-navigation \
	&& apt-get install -y ros-noetic-turtlebot3-simulations \
	&& apt-get install -y ros-noetic-turtlebot3-slam ros-noetic-turtlebot3-teleop \
	&& apt-get install -y ros-noetic-gmapping ros-noetic-slam-gmapping ros-noetic-openslam-gmapping \ 
	&& pip3 install powerline-shell  \
	&& rm -rf /var/lib/apt/lists/* \
	&& useradd -ms /bin/bash user \
	&& echo "user:user" | chpasswd && adduser user sudo \
	&& echo "user ALL=(ALL) NOPASSWD: ALL " >> /etc/sudoers

# The OSRF contianer didn't link python3 to python, causing ROS scripts to fail.
RUN ln -s /usr/bin/python3 /usr/bin/python

USER user
WORKDIR /home/user
RUN sudo usermod -a -G video user

RUN git clone https://github.com/jimeh/tmux-themepack.git ~/.tmux-themepack  \
        && git clone https://github.com/tmux-plugins/tmux-resurrect ~/.tmux-resurrect
COPY --chown=user:user ./.tmux.conf /home/user/.tmux.conf
COPY --chown=user:user ./.powerline.sh /home/user/.powerline.sh

RUN mkdir -p /home/user/.vim/bundle \
        && git clone https://github.com/VundleVim/Vundle.vim.git ~/.vim/bundle/Vundle.vim

COPY --chown=user:user ./.vimrc /home/user/.vimrc
 
RUN set -x \
        && vim -E -u NONE -S /home/user/.vimrc -C "+PluginInstall" -C "+qall";  exit 0
#
# Set some decent colors if the container needs to be accessed via /bin/bash.
RUN echo LS_COLORS=$LS_COLORS:\'di=1\;33:ln=36\' >> ~/.bashrc \
        && echo export LS_COLORS >> ~/.bashrc \
        && echo 'source ~/.powerline.sh' >> ~/.bashrc \
        && echo 'alias tmux="tmux -2"' >> ~/.bashrc \
        && echo 'PATH=~/bin:$PATH' >> ~/.bashrc \
        && touch ~/.sudo_as_admin_successful # To surpress the sudo message at run.

RUN rosdep update \
        && echo "source /opt/ros/noetic/setup.bash" >> /home/user/.bashrc

RUN mkdir -p Projects/catkin_ws/src 
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/Projects/catkin_ws; catkin_make'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/Projects/catkin_ws; catkin_make'

RUN echo "source /home/user/Projects/catkin_ws/devel/setup.bash --extend" >> /home/user/.bashrc
STOPSIGNAL SIGTERM

CMD sudo service ssh start && /bin/bash
