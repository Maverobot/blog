+++
title = "Setup ROS 2 Development Environment with Docker and Emacs on MacOS"
author = ["Zheng Qu"]
date = 2024-01-11T00:00:00+01:00
lastmod = 2024-01-12T19:38:28+01:00
tags = ["ROS2", "Emacs", "Docker", "MacOS"]
draft = false
weight = 2001
+++

Setting up the right environment is crucial for effective development. In this
blog post, I will guide you through setting up a [ROS 2](https://docs.ros.org/en/rolling/index.html) development environment
using [Docker](https://www.docker.com) and [Emacs](https://www.gnu.org/software/emacs/download.html) on [MacOS](https://www.apple.com/de/macos/sonoma/). This guide assumes that Emacs is already
installed on your system.


## Install Docker {#install-docker}

You need to install [Homebrew](https://brew.sh) first. After that, you can install Docker with:

```sh
brew install --cask docker
```

Start the docker app to bring up the docker daemon.

Now you can check if the installation was successful by running the following command:

```sh
docker run hello-world
```


## Add your ROS 2 workspace {#add-your-ros-2-workspace}

Add a workspace in order to build and open them in a container, e.g.:

```sh
cd ~/
mkdir -p ws_[project]/src
```

Now create a `.devcontainer` folder in the root of your workspace and add a `devcontainer.json` and `Dockerfile` to this `.devcontainer` folder.
Additionally, you need to create a `cache` folder in which you can cache the `build`, `install` and `log` folders for different ROS 2 distros.
The workspace structure should look like this:

```sh
ws_[project]
├── cache
|   ├── [ROS2_DISTRO]
|   |   ├── build
|   |   ├── install
|   |   └── log
|   └── ...
|
├── src
    ├── .devcontainer
    │   ├── devcontainer.json
    │   └── Dockerfile
    ├── package1
    └── package2
```


## Edit `devcontainer.json` {#edit-devcontainer-dot-json}

For the Dev Container to function properly, we have to build it with the correct user.
Therefore add the following to `.devcontainer/devcontainer.json`:

```json
{
  "name": "ROS 2 Development Container",
  "privileged": true,
  "remoteUser": "YOUR_USERNAME",
  "build": {
    "dockerfile": "Dockerfile",
    "args": {
      "USERNAME": "YOUR_USERNAME"
    }
  },
  "workspaceFolder": "/home/ws",
  "workspaceMount": "source=${localWorkspaceFolder},target=/home/ws/src,type=bind",
  "customizations": {
    "vscode": {
      "extensions":[
        "ms-vscode.cpptools",
        "ms-vscode.cpptools-themes",
        "twxs.cmake",
        "donjayamanne.python-extension-pack",
        "eamodio.gitlens",
        "ms-iot.vscode-ros"
      ]
    }
  },
  "containerEnv": {
    "DISPLAY": "unix:0",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    "ROS_DOMAIN_ID": "42"
  },
  "runArgs": [
    "--net=host",
    "-e", "DISPLAY=host.docker.internal:0"
  ],
  "mounts": [
    "source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind,consistency=cached",
    "source=${localWorkspaceFolder}/../cache/ROS_DISTRO/build,target=/home/ws/build,type=bind",
    "source=${localWorkspaceFolder}/../cache/ROS_DISTRO/install,target=/home/ws/install,type=bind",
    "source=${localWorkspaceFolder}/../cache/ROS_DISTRO/log,target=/home/ws/log,type=bind"
  ],
  "postCreateCommand": "sudo rosdep update && sudo rosdep install --from-paths src --ignore-src -y && sudo chown -R YOUR_USERNAME /home/ws/"
}
```

Open the file in your editor, search for `YOUR_USERNAME` and replace it with your username.
If you do not know your username, you can find it by running `echo $USER` in the terminal.
Also replace `ROS_DISTRO`, with the ROS 2 distribution that you want to use and added to the cache previously, for example, `humble` or `rolling`.


## Edit Dockerfile {#edit-dockerfile}

Open the `Dockerfile` and add the following contents:

```dockerfile
FROM ros:ROS_DISTRO
ARG USERNAME=YOUR_USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME
RUN useradd --uid $USER_UID --gid $USER_GID -m $USERNAME
#
# [Optional] Add sudo support. Omit if you don't need to install software after connecting.
RUN apt-get -y update
RUN apt-get install -y sudo
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME
RUN chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y

# Install a few important dev dependencies
RUN apt-get install -y \
    ament-cmake \
    ccls \
    python3-colcon-common-extensions \
    python3-pip \
    vim \
    clang \
    clang-format \
    clang-tidy

RUN echo "source /opt/ros/ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]
```

Search here also for the `YOUR_USERNAME` and replace it with your username and the
`ROS_DISTRO` with the ROS 2 distribution you wish to use and added to the cache
previously.


## Open and Build Devcontainer {#open-and-build-devcontainer}

To build and start the [devcontainer](https://code.visualstudio.com/docs/devcontainers/containers), we need [GitHub - devcontainers/cli](https://github.com/devcontainers/cli). Install
it with:

```sh
brew install devcontainer
```

Afterwards, execute the following commands to build and start the devcontainer:

```sh
devcontainer up --workspace-folder ~/ws_[project]/src
```


## Build the ROS 2 workspace {#build-the-ros-2-workspace}

To build the ROS 2 workspace, run

```sh
devcontainer exec --workspace-folder ~/ws_[project]/src colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```


## Edit files with emacs {#edit-files-with-emacs}

Open the file you want to edit with `helm-find-files`, type `/docker:` and press `TAB` to auto-complete the docker container name.
Then you can find the file you want to edit in the container with path like:

```sh
/docker:container_name:/home/ws/src/package1/src/file.cpp
```

Once the file is opened, you can edit it as usual. `ccls` or `clangd` will automatically index the file if

-   it is configured in your emacs configuration,
-   the `compile_commands.json` is present in the build folder of the ROS 2 workspace,
-   and the executable `ccls` or `clangd` is installed in the container.


## Test ROS 2 CLI {#test-ros-2-cli}

Here we will follow the [turtlesim, ros2, and rqt — ROS 2 Tutorial](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html).


### Install turtlesim {#install-turtlesim}

First, we need to install the turtlesim package. Open a new terminal and run:

```sh
devcontainer exec --workspace-folder ~/ws_[project]/src bash
```

which gives you a bash shell in the container. Then in the bash shell, run:

```sh
sudo apt install ros-ROS_DISTRO-turtlesim
```

Now check if the installation was successful by running:

```sh
ros2 pkg executables turtlesim
```


### Start turtlesim {#start-turtlesim}

To start turtlesim, you can either run:

-   `ros2 run turtlesim turtlesim_node` in the bash shell of the container, or
-   `devcontainer exec --workspace-folder ~/ws_[project]/src ros2 run turtlesim turtlesim_node` in a new terminal.

Now you may encounter an error saying

```nil
Error: Can't open display: host.docker.internal:0
```

To solve it you need to follow the steps here:

-   Install [XQuartz](https://www.xquartz.org) with `brew install --cask xquartz`.
-   Open XQuartz and go to `Preferences` -&gt; `Security` and check `Allow connections from network clients`.
-   Reboot the computer.
-   Run `xhost +localhost` in a terminal to allow connections to the MacOS host.

Now you would be able to start turtlesim and see a "TurtleSim" window pop up.


### Use TurtleSim {#use-turtlesim}

To control the turtle, you can either run:

-   `ros2 run turtlesim turtle_teleop_key` in the bash shell of the container, or
-   `devcontainer exec --workspace-folder ~/ws_[project]/src ros2 run turtlesim turtle_teleop_key` in a new terminal.

Now you can use the arrow keys to control the turtle in the "TurtleSim" window. And you should be able to continue
the [tutorial](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html) to learn more about ROS 2 CLI.


## References {#references}

-   [Setup ROS 2 with VSCode and Docker](https://docs.ros.org/en/iron/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html)
-   [Dev Containers Part 2: Setup, the devcontainer CLI &amp; Emacs](https://happihacking.com/blog/posts/2023/dev-containers-emacs/)
-   [Running X apps on MacOS with docker](http://mamykin.com/posts/running-x-apps-on-mac-with-docker/)
