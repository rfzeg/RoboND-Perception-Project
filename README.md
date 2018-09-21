[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# 3D Perception
Before starting any work on this project, please complete all steps for [Exercise 1, 2 and 3](https://github.com/udacity/RoboND-Perception-Exercises). At the end of Exercise-3 you have a pipeline that can identify points that belong to a specific object.

In this project, you must assimilate your work from previous exercises to successfully complete a tabletop pick and place operation using PR2.

The PR2 has been outfitted with an RGB-D sensor much like the one you used in previous exercises. This sensor however is a bit noisy, much like real sensors.

Given the cluttered tabletop scenario, you must implement a perception pipeline using your work from Exercises 1,2 and 3 to identify target objects from a so-called “Pick-List” in that particular order, pick up those objects and place them in corresponding dropboxes.

PLACEHOLDER

**Video of the completed pick and place process**

#### Table of Contents
1. Development Process
   1. Decouple 
   2. Inverse 
   3. Inverse 
2. Extra Challenge: Complete Pick & Place
3. Future Improvements
4. Project Setup

Extra Challenges: Complete Pick & Place

# Development Process
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify).  
**Generate Features:**
To generate features, first launch the training.launch file to bring up the Gazebo environment:
```$ roslaunch sensor_stick training.launch```  
Next, in a new terminal, run the capture_features.py script to capture and save features for each of the objects in the environment.
```$ rosrun sensor_stick capture_features.py```  
**Note:** The training_set.sav file will be saved in the current directory, where the script was executed.  
This script spawns each object in 20 random orientations.  
The script was modified to include following models:
```
- biscuits
- book
- eraser
- glue
- snacks
- soap
- soap2
- sticky_notes
```

After that you run the train_svm.py script to train an SVM classifier on your labeled set of features.
``` $ rosrun sensor_stick train_svm.py ```

**Training Results:**
``` 
Features in Training Set: 160  
Invalid Features in Training set: 0  
Scores: [ 0.9375   0.96875  0.84375  0.90625  0.9375 ]  
Accuracy: 0.92 (+/- 0.08)  
accuracy score: 0.91875  
```

The following plots show the relative accuracy of the classifier for the various objects:

![demo-1](https://github.com/digitalgroove/RoboND-Perception-Project/blob/master/writeup_images/SVM-Confusion-Matrix-Perception-Project.png)
**Image: Confusion matrix**

Running the above command will also result in your trained model being saved in a model.sav file.
**Note:** This model.sav file will be saved in the current directory, where the script was executed.

**Note:** keep in mind that the model.sav file needs to be in the same directory where you run this!
**Note2:** chmod +x project_template.py

Copy the model.sav to the directory where the project_template.py script is located.

To test with the development so far, first run:
``` $ roslaunch pr2_robot pick_place_project.launch ```

and then in another terminal, run your object recognition node:
**IMPORTANT:** RUN THE NEXT SCRIPT FROM THE DIRECTORY WHERE THE FILE MODEL.SAV IS LOCATED:
e.g first $ cd ~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/scripts
``` $ rosrun pr2_robot project_template.py ```


2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Remove outliers using a statistical outlier filter

To clean up the noise of the camera image we use the statistical outlier filter found in python-pcl:
![demo-1](https://github.com/digitalgroove/RoboND-Perception-Project/blob/master/writeup_images/perception-project-noisy-cloud.png)
**Image: Point cloud before statistical outlier filtering**

```python
    ## Statistical Outlier Filtering
    # Much like the previous filters, we start by creating a filter object: 
    outlier_filter = pcl_data.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(10)

    # Set threshold scale factor
    # All points who have a distance larger than X standard deviation of the mean distance
    # to the query point will be marked as outliers and removed
    x = 0.01

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)

    # Finally call the filter function for magic
    cloud_outlier_filtered = outlier_filter.filter()
``` 

![demo-1](https://github.com/digitalgroove/RoboND-Perception-Project/blob/master/writeup_images/perception-project-filtered-cloud.png)
**Image: Point cloud after statistical outlier filtering**

4. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.  
This part of the code was mainly developed during Perception Exercise-2.
```python 
    ## RANSAC plane segmentation (was TODO)
    # Identifies points that belong to a particular model (plane, cylinder, box, etc.)

    # Create the segmentation object
    seg = cloud_passthrough_2.make_segmenter()

    # Set the model you wish to fit 
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance 
    # for segmenting the table
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()
```
5. Apply Euclidean clustering to create separate clusters for individual items.
```python
    ## Euclidean Clustering
    # Apply function to convert XYZRGB to XYZ
    white_cloud = XYZRGB_to_XYZ(cloud_objects) 
    tree = white_cloud.make_kdtree() # returns a kd-tree

    # Perform the cluster extraction
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(5000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
```
6. Perform object recognition on these objects and assign them labels (markers in RViz).  
This part of the code was mainly developed during Perception Exercise-3.
```python
    ## Perform object recognition on these objects and assign them labels
    # Create some empty lists to receive labels and object point clouds
    detected_objects_labels = []
    detected_objects = []

    # Classify the clusters! (loop through each detected cluster one at a time)

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted objects (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        # Convert the cluster from pcl to ROS using helper function (was TODO)
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Extract histogram features (Compute the associated feature vector)
        # Complete this step just as is covered in capture_features.py (was TODO)
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects
        # Declare a object of message type DetectedObject:
        do = DetectedObject() 
        do.label = label
        do.cloud = ros_cluster
        # A list of detected objects (of message type DetectedObject)
        detected_objects.append(do)

    ## end of for loop ##
    # Prints list of detected objects:
    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)
```
![demo-1](https://github.com/digitalgroove/RoboND-Perception-Project/blob/master/writeup_images/perception-project-world3-results.png)
**Image: Point cloud after object recognition and object labeling tested in World 3**


7. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
```python
    ## Loop through the pick list (was TODO)
    # Loop over our list using a plain for-in loop
    for index, item in enumerate(object_list_param):

        print "========== NEW ITERATION OVER PICK UP LIST ==========" # for debugging      
        ## Parse parameters into individual variables (was TODO)
        # object_list_param can be parsed to obtain object names and associated group
        object_name = object_list_param[index]['name']
        object_group = object_list_param[index]['group'] # will be either green or red
        # print object_name # for debugging

        ## Get the PointCloud for a given object and obtain it's centroid (was TODO)
        try:
            # Find the position of the current object from the pick up list inside the list of detected_objects
            # Get index in a list of objects by attribute            
            idx = [ x.label for x in object_list ].index(object_name)
        except ValueError:
            print "Object in pick-up list was not detected: {}".format(object_name)
            print "Continue with other objects in pick up list."
            continue
        print "Object to pick up: {}".format(object_name) # for debugging
        print "Index in list of detected objects (object_list): {}".format(idx) # for debugging
        # Use that position to retrieve the associated point cloud of the object
        pcl = object_list[idx].cloud
        labels.append(object_name)
        points_arr = ros_to_pcl(pcl).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])
```

8. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  
```python
        ## Pick pose, calculated pose of recognized object's centroid (was TODO)
        # Initialize an empty pose message
        pick_pose = Pose()

        # Fill in appropriate fields, access last object in centroids list
        # Recast to native Python float type using np.asscalar()
        pick_pose.position.x = np.asscalar(centroids[-1][0])
        pick_pose.position.y = np.asscalar(centroids[-1][1])
        pick_pose.position.z = np.asscalar(centroids[-1][2])
        print "Centroid/Position of last object added to list: {0}, {1}, {2}".format(pick_pose.position.x,pick_pose.position.y,pick_pose.position.z) # for debugging
     
        ## Create and fill in message variable for the name of the object (was TODO)
        # Initialize a variable
        msg_object_name = String()
        # Populate the data field
        msg_object_name.data = object_name

        ## Get the position for a given dropbox (same as other TODO)
        # Search a list of dictionaries and return a selected value in selected dictionary 
        selected_entry = [item for item in dropbox_obj_param if item['group'] == object_group][0]
        dropbox_position = selected_entry.get('position')
        print "Position extracted from yaml file: {}".format(dropbox_position) # for debugging
      
        ## Create 'place_pose' or object placement pose for the object (was TODO)
        # Initialize an empty pose message
        place_pose = Pose()
        # Fill in appropriate fields
        place_pose.position.x = dropbox_position[0]
        place_pose.position.y = dropbox_position[1]
        place_pose.position.z = dropbox_position[2] 

        ## Assign the arm to be used for pick_place (was TODO)
        # Name of the arm can be either right/green or left/red
        # Initialize a variable
        which_arm = String()
        # Populate the data field
        if object_group == 'green':
            which_arm.data = 'right' 
        else:
            which_arm.data = 'left'
        print "Arm: {}".format(which_arm.data) # for debugging

        ## Create and fill in other message variables

        ## The test scene number (either 1, 2 or 3) (was TODO)
        # Initialize the test_scene_num variable
        test_scene_num = Int32()
        # Get/Read parameters of dropbox positions
        test_scene = rospy.get_param('/test_scene_num') # parameter name
        # Populate the data field of that variable:
        test_scene_num.data = test_scene
        print "Test Scene Number: {}".format(test_scene_num.data) # for debugging

        ## Populate various ROS messages
        yaml_dict = make_yaml_dict(test_scene_num, which_arm, msg_object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')
```
See the block below for details on how the output (output_1.yaml) looks like:
```

```


# Extra Challenge: Complete Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

For all the step-by-step details on how to complete this project see the [RoboND 3D Perception Project Lesson](https://classroom.udacity.com/nanodegrees/nd209/parts/586e8e81-fc68-4f71-9cab-98ccd4766cfe/modules/e5bfcfbd-3f7d-43fe-8248-0c65d910345a/lessons/e3e5fd8e-2f76-4169-a5bc-5a128d380155/concepts/802deabb-7dbb-46be-bf21-6cb0a39a1961)
Note: The robot is a bit moody at times and might leave objects on the table or fling them across the room :D
As long as your pipeline performs succesful recognition, your project will be considered successful even if the robot feels otherwise!


# Future Improvements
- write output file once and stop

- improove inverse kinematics, avoid swirling of arm, 
- improve logic of the gripper (e.g. use orientation data)
- use other detected model if not in pick up list,  use class membership probability estimates

# Project Setup
For this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly
If you do not have an active ROS workspace, you can create one by:

```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the src directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Perception-Project.git
```
### Note: If you have the Kinematics Pick and Place project in the same ROS Workspace as this project, please remove the 'gazebo_grasp_plugin' directory from the `RoboND-Perception-Project/` directory otherwise ignore this note. 

Now install missing dependencies using rosdep install:
```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```
Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/models:$GAZEBO_MODEL_PATH
```

If you haven’t already, following line can be added to your .bashrc to auto-source all new terminals
```
source ~/catkin_ws/devel/setup.bash
```

To run the demo:
```sh
$ cd ~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/scripts
$ chmod u+x pr2_safe_spawner.sh
$ ./pr2_safe_spawner.sh
```
![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)



Once Gazebo is up and running, make sure you see following in the gazebo world:
- Robot

- Table arrangement

- Three target objects on the table

- Dropboxes on either sides of the robot


If any of these items are missing, please report as an issue on [the waffle board](https://waffle.io/udacity/robotics-nanodegree-issues).

In your RViz window, you should see the robot and a partial collision map displayed:

![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Proceed through the demo by pressing the ‘Next’ button on the RViz window when a prompt appears in your active terminal

The demo ends when the robot has successfully picked and placed all objects into respective dropboxes (though sometimes the robot gets excited and throws objects across the room!)

Close all active terminal windows using **ctrl+c** before restarting the demo.

You can launch the project scenario like this:
```sh
$ roslaunch pr2_robot pick_place_project.launch
```
