
if [ $# = 0 ]; then
    grabside="pr1012"
elif [ $1 = "pr1012" ]; then
    grabside="pr1012"
elif [ $1 = "pr1040" ]; then
    grabside="pr1040"
else
    echo "Mismatch argument!"
    return 1
fi

tmux-newwindow() {
    if [ `tmux list-windows | grep $1 | sed -e 's/ //g'` ]; then
        echo $1 "already exists"
    else
        tmux new-window -k -n $1 -t drc
        tmux send-keys -t drc:$1 "$2" C-m
    fi
}

# if [ $# -gt 0 ]; then
#     if [ $1 = "--kill" -o $1 = "k" ]; then
#         tmux kill-session -t drc
#         exit 0
#     elif [ $1 = "--attach" -o $1 = "a" ]; then
#         tmux a -t drc
#         exit 0
#     fi
# fi

if `tmux has-session -t drc`; then
    echo -e "\e[1;33msession named drc already exists.\e[m"
else
    echo -e "\e[1;34mcreate new session named drc.\e[m"
    tmux new-session -d -s drc -n tmp
fi

# tmux-newwindow pr1012 "sleep 1; pr1012; roslaunch tableclothdemo tableclothdemo_pr1012_rocon.launch"
# tmux-newwindow grab-side "sleep 1; pr1012; cd euslisp; roseus grab-side.l"
# tmux-newwindow pr1040 "sleep 1; pr1040; roslaunch tableclothdemo tableclothdemo_pr1040_rocon.launch"
# tmux-newwindow otherside "sleep 1; pr1040; cd euslisp; roseus grab-otherside.l"

if [ $grabside = "pr1012" ]; then
    echo "pr1012:grabside pr1040:otherside"
    tmux-newwindow pr1012 "sleep 1; pr1012; roslaunch tableclothdemo pr1012_grabside.launch"
    tmux-newwindow grab-side "sleep 1; pr1012; cd euslisp; roseus grab-side.l"
    tmux-newwindow pr1040 "sleep 1; pr1040; roslaunch tableclothdemo pr1040_otherside.launch"
    tmux-newwindow otherside "sleep 1; pr1040; cd euslisp; roseus grab-otherside.l"
elif [ $grabside = "pr1040" ]; then
    echo "pr1040:grabside pr1012:otherside"
    echo "pr1040"
    tmux-newwindow pr1012 "sleep 1; pr1012; roslaunch tableclothdemo pr1012_otherside.launch"
    tmux-newwindow otherside "sleep 1; pr1012; cd euslisp; roseus grab-otherside.l"
    tmux-newwindow pr1040 "sleep 1; pr1040; roslaunch tableclothdemo pr1040_grabside.launch"
    tmux-newwindow grab-side "sleep 1; pr1040; cd euslisp; roseus grab-side.l"
else
    echo "Mismatch argument!"
fi
