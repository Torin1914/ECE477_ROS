#!/bin/bash

# Start ROScore
roscore &

# Wait for ROScore to initialize
sleep 5

# Start nodes
gnome-terminal --tab --title="Ball Detection" --command="bash -c 'source /opt/ros/melodic/setup.bash; rosrun fetch_bot ball_detection.py; exec bash'" &
gnome-terminal --tab --title="Fetch Ball" --command="bash -c 'source /opt/ros/melodic/setup.bash; rosrun fetch_bot fetch_ball.py; exec bash'" &
gnome-terminal --tab --title="UART" --command="bash -c 'source /opt/ros/melodic/setup.bash; rosrun fetch_bot UART.py; exec bash'" &
gnome-terminal --tab --title="Return to Sender" --command="bash -c 'source /opt/ros/melodic/setup.bash; rosrun fetch_bot return_to_sender.py; exec bash'" &

# Keep the script running
wait
