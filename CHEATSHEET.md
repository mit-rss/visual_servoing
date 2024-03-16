# Cheatsheet

## Homography Transformation
### Setup
- Connect to the racecar's docker image (`ssh racecar` -> `connect` -> `cd racecar_ws`)
- Build (`colcon build && source install/setup.bash`)
- Launch the camera (`ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed`)
    - If it errors with something about a display, run `unset DISPLAY` before this command

### Visualizing Stuff
- Launching noVNC:
    - Locally, *not* on the racecar: `ssh -L 6081:localhost:6081 racecar@192.168.1.74`
    - Open [noVNC](http://localhost:6081/vnc.html?resize=remote) in your web browser
- Open a terminal, enter `rqt` and go to `Plugins > Visualization > Image View`
- Change the subscription topic (left-most dropdown) to `/zed/zed_node/left/image_rect_color`
    - You may need to hit "refresh topics" (button to the right of the dropdown) if it's not working
- Change the publishing topic (text box below the dropdown) to `/zed/rgb/image_rect_color_mouse_left` and click the checkmark to its left
    - This enables publishing left mouse clicks' positions to that topic
- Open a terminal, enter `rviz2`