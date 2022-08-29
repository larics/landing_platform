#!/bin/bash

DOCKERFILE=Dockerfile
DISTRO=bionic
build_args=""

for (( i=1; i<=$#; i++));
do
  param="${!i}"
  echo $param

  if [ "$param" == "--bionic" ]; then
    DISTRO="bionic"
  fi

  if [ "$param" == "--focal" ]; then
    DISTRO="focal"
  fi

  if [ "$param" == "--dockerfile" ]; then
    j=$((i+1))
    DOCKERFILE="${!j}"
  fi

  if [ "$param" == "--build-args" ]; then
    j=$((i+1))
    build_args="${!j}"
  fi

done

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

echo "Building landing_platform:$DISTRO distrowith additional docker arguments $build_args."
echo "Dockerfile: $DOCKERFILE"

# export BUILDKIT_PROGRESS=plain
export DOCKER_BUILDKIT=1
docker build \
    $build_args \
    --build-arg DISTRO=$DISTRO \
    -f $MY_PATH/$DOCKERFILE \
    --ssh default \
    -t landing_platform:focal $MY_PATH/..
