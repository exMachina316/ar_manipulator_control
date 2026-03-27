# Getting all transforms using TF2 CLI

To display all transformations in ROS 2 tf, you can use command-line tools like ros2 run tf2_tools view_frames.py for a visual tree or ros2 run tf2_ros tf2_echo for a specific transform, or visualize them in RViz 2. 
Command-line Tools
These tools are useful for quick checks and debugging directly in the terminal:
view_frames.py: Generates a PDF file (frames.pdf) that visualizes the entire TF tree structure, including the parent-child relationships, the names of the frames, and diagnostic information like the publication rate.
bash
ros2 run tf2_tools view_frames.py
# A frames.pdf file will be generated in your current directory.
tf2_echo: Reports the real-time translation and rotation (both in quaternion and RPY format) between any two specified frames.
bash
ros2 run tf2_ros tf2_echo [source_frame] [target_frame]
# Example:
ros2 run tf2_ros tf2_echo world base_link
ros2 topic echo: You can also manually inspect the raw tf and tf_static messages being published on the network, although it can be difficult to interpret the continuous stream of messages.
bash
ros2 topic echo /tf
ros2 topic echo /tf_static
 
Visualization with RViz 2
For a dynamic and interactive 3D visualization, RViz 2 is the recommended tool:
Run RViz 2:
bash
ros2 run rviz2 rviz2
Add the TF display: In the RViz 2 interface, click the "Add" button in the bottom-left "Displays" panel, then select the "TF" display type.
Set the Fixed Frame: In the "Displays" panel, find the "Global Options" section and set the "Fixed Frame" to a common frame in your system (e.g., world or map).
Configure display options: In the TF display settings, you can enable options like "Show Names", "Show Axes", and "Show Arrows" to better visualize the relationships and movement of all broadcast frames. 
As transformations are published over the ROS 2 network, the frames will appear and move in real-time within the 3D visualization panel of RViz 2. 
