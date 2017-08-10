# perception_driven

Not full files for build

v0.3 - Listening for fixation_3d topic, from track_3d_gaze_node. Using the 3D fixation as input. 

v0.2 - Added compute cartesian path for moving to drink position (method 1)
       Added orientation constraint for moving to drink position (method 2)
       Changed planner from LBKPIECE to RRTConnectkConfigDefault to find solutions

v0.1 - Click point cloud to obtain, compare with ORK cloud in a radius to check if there are nearby points. If there is, compare centroid pose, the closest is the object selected. Pose of object will be sent to arm, arm goes to prepare position (offseted z), picks up object then takes to drink position, drink then unload object.
