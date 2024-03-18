#!/bin/bash

if [ $# -eq 0 ]; then
    echo "Usage: $(basename "$0") <screen_definition_file>"
    exit 1
fi

SCREEN_DEF_FILE=$1

if [ ! -f "$SCREEN_DEF_FILE" ]; then
    echo "File not found: $SCREEN_DEF_FILE"
    exit 1
fi

source "$SCREEN_DEF_FILE"

install() {
    #
    # find location for 'install/setup.bash'
    #
    local DIR="$(pwd)"
    while [ "$DIR" != '/' ]; do
        if [ -d "$DIR/install" ]; then
            echo "$DIR/install"
            return
        fi
        DIR=$(dirname "$DIR")
    done
    echo "folder 'install' not found" >&2
    exit 1
}

ROS_DISTRO=$( ( [ -d /opt/ros/galactic/ ] && echo galactic ) ||  ([ -d /opt/ros/humble/ ] && echo humble ) )
[ -z "$ROS_DISTRO" ] && echo "Neither humble nor galactic installed" >&2  && exit 1
export ROS_DISTRO
export ROS_DOMAIN_ID=100


cmd='clear'
cmd="$cmd && source /opt/ros/${ROS_DISTRO}/setup.bash"
if [ -d /opt/bautiro/btr_external_ws ]; then
    cmd="$cmd && source /opt/bautiro/btr_external_ws/install/setup.bash"
fi
cmd="$cmd && source $(install)/setup.bash"
cmd="$cmd && "


[ -z "$engine" ] && engine='byobu'
if [ "$engine" = 'tmux' ]; then V=" C-z "; N=" Enter "; else N=" C-m "; fi

echo using backend: $engine
sleep 2
terminal_split_windows_and_panes() {
    local commands=("$@")
    $engine new-window -n "$window_name"

    for i in "${!commands[@]}"; do
        if [ $i -eq 0 ]; then
            $engine send-keys $V "$cmd ${commands[$i]}" $N
        else
            $engine split-window -v
            $engine select-layout even-vertical
            $engine send-keys $V "$cmd ${commands[$i]}" $N
        fi
    done
}

SN='mysession'
$engine kill-session -t $SN
$engine new-session -d -s $SN

for window_name in "${windows[@]}"; do
    declare -n screen_ref="$window_name"
    terminal_split_windows_and_panes "${screen_ref[@]}"
done

$engine attach-session -t $SN
