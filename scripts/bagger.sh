#!/bin/bash
echo "Bagger script: First argument is filename, rest are topic names!"
echo "Filename: ${1}"
echo "Topcis: ${@:2}"

topic_list=$(rostopic list 2>/dev/null;) 
if [ -z "$topic_list" ]
then
  echo "No available topics to record, exiting!"
  exit 1
fi

echo "Available topics:"
echo $topic_list

valid_topics=""
for topic in "${@:2}"
do
  if [[ ! $topic = /* ]];
  then
    echo "IGNORE: $topic!"
    continue
  fi

  check=$(echo $topic_list | grep -w "$topic")
  if [ ! -z "$check" ] && [ $? -eq 0 ];
  then
    echo "Found topic: $topic"
    valid_topics="$topic $valid_topics"
  else
    echo "No topic: $topic. Exiting..."
    exit 2
  fi
done

if [ -z "$valid_topics" ]
then
  echo "No valid topics found, exiting!"
  exit 1
fi

dir_path=$(dirname ${1})
mkdir -p $dir_path

rosbag record -o ${1} $valid_topics
