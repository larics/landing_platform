#!/bin/bash

# Which GPUs to use; see https://github.com/NVIDIA/nvidia-docker
#GPUS="all"
GPUS=""

# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# It still not working, try running the script as root.

XAUTH=/tmp/.docker.xauth

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi


CONTAINER_NAME=$1
[ -z "$CONTAINER_NAME" ] && CONTAINER_NAME=landing_platform

IMAGE_NAME=$2
[ -z "$IMAGE_NAME" ] && IMAGE_NAME=landing_platform:bionic

# Hook to the current SSH_AUTH_LOCK - since it changes
# https://www.talkingquickly.co.uk/2021/01/tmux-ssh-agent-forwarding-vs-code/
ln -sf $SSH_AUTH_SOCK ~/.ssh/ssh_auth_sock

docker run \
  -it \
  --network host \
  --privileged \
  --volume /dev:/dev \
  --volume $HOME/bags:/root/bags \
  --volume $HOME/rosbag_default:/root/rosbag_default \
  --volume ~/.ssh/ssh_auth_sock:/ssh-agent \
  --env SSH_AUTH_SOCK=/ssh-agent \
  --volume $XSOCK:$XSOCK:rw \
  --volume $XAUTH:$XAUTH:rw \
  --env XAUTHORITY=${XAUTH} \
  --env DISPLAY=$DISPLAY \
  --env TERM=xterm-256color \
  --env QT_X11_NO_MITSHM=1 \
  --name $CONTAINER_NAME \
  $IMAGE_NAME \
  /bin/bash
