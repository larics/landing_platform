#!/bin/bash

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

# remove the old link
rm .tmuxinator.yml

# link the session file to .tmuxinator.yml
# ln session_ar_tag_detection.yml .tmuxinator.yml
# ln session_sim_platform_landing.yml .tmuxinator.yml
ln session_rw_platform_landing.yml .tmuxinator.yml


# start tmuxinator
tmuxinator
