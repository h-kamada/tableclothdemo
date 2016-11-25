
tmux-newwindow() {
    if [ `tmux list-windows | grep $1 | sed -e 's/ //g'` ]; then
        echo $1 "already exists"
    else
        tmux new-window -k -n $1 -t drc
        tmux send-keys -t drc:$1 "$2" C-m
    fi
}

if [ $# -gt 0 ]; then
    if [ $1 = "--kill" -o $1 = "k" ]; then
        tmux kill-session -t drc
        exit 0
    elif [ $1 = "--attach" -o $1 = "a" ]; then
        tmux a -t drc
        exit 0
    fi
fi

if `tmux has-session -t drc`; then
    echo -e "\e[1;33msession named drc already exists.\e[m"
else
    echo -e "\e[1;34mcreate new session named drc.\e[m"
    tmux new-session -d -s drc -n tmp
fi

tmux-newwindow pr1012 "sleep 1; pr1012; roslaunch tableclothdemo chatter_pr1012_rocon.launch"
tmux-newwindow pr1040 "sleep 1; pr1040; roslaunch tableclothdemo chatter_pr1040_rocon.launch"



