#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject # message type DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf  # or from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose # for sending the pick pose and place pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32 # for sending the test scene number
from std_msgs.msg import String # for sending the object name
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    print ""
    print "========== RECEIVED NEW POINT CLOUD MESSAGE ==========" # for debugging

# Exercise-2 TODOs:

    ## Convert ROS msg to PCL data (was TODO)
    # Takes in a ROS message of type PointCloud2 and converts it to PCL PointXYZRGB format
    pcl_data = ros_to_pcl(pcl_msg)

    ## Statistical Outlier Filtering (was TODO)
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

    ## Voxel Grid filter
    # A voxel grid filter allows to downsample the data

    # Create a VoxelGrid filter object for our input point cloud
    vox = cloud_outlier_filtered.make_voxel_grid_filter()

    # Choose a voxel (also known as leaf) size
    # Note: using 1 is a poor choice of leaf size
    # Units of the voxel size (or leaf size) are in meters
    # Experiment and find the appropriate size!
    LEAF_SIZE = 0.005

    # Set the voxel (or leaf) size  
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()

    ## PassThrough filter (was TODO)
    # It allows to crop a part by specifying an axis and cut-off values

    # Create a PassThrough filter object.
    passthrough = cloud_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    # Here axis_min and max is the height with respect to the ground
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6 # to retain only the tabletop and the objects sitting on the table
    axis_max = 1.0 # to filter out the upper part of the cloud
    passthrough.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_passthrough_1 = passthrough.filter()

    # Create a PassThrough filter object.
    passthrough_x = cloud_passthrough_1.make_passthrough_filter()
    # Assign axis and range to the passthrough filter object.
    # Here axis_min and max is the height with respect to the ground
    filter_axis = 'x'
    passthrough_x.set_filter_field_name(filter_axis)
    x_axis_min = 0.35 # to filter out the closest part of the cloud
    x_axis_max = 1.5 # to filter out the farest part of the cloud
    passthrough_x.set_filter_limits(x_axis_min, x_axis_max)
    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_passthrough_2 = passthrough_x.filter()


    ## RANSAC plane segmentation (was TODO)
    # Identifys points that belong to a particular model (plane, cylinder, box, etc.)

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

    ## Extract inliers and outliers (was TODO)
    # Allows to extract points from a point cloud by providing a list of indices
    cloud_table = cloud_passthrough_2.extract(inliers, negative=False)
    # With the negative flag True we retrieve the points that do not fit the RANSAC model
    cloud_objects = cloud_passthrough_2.extract(inliers, negative=True)
    
    ## Euclidean Clustering (was TODO)
    white_cloud = XYZRGB_to_XYZ(cloud_objects) # Apply function to convert XYZRGB to XYZ
    tree = white_cloud.make_kdtree()

    # Perform the cluster extraction
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # NOTE: 0.001, 10 and 250 are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(5000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    ## Create Cluster-Mask Point Cloud to visualize each cluster separately (was TODO)
    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    # Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    ## Convert PCL data to ROS messages (was TODO)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    # Cloud containing all clusters (objects), each with unique color:
    ros_cluster_cloud = pcl_to_ros(cluster_cloud) 
    ros_outlier_filtered_cloud = pcl_to_ros(cloud_outlier_filtered)

    ## Publish ROS messages (was TODO)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    # Cloud containing all clusters (objects), each with unique color:
    pcl_cluster_pub.publish(ros_cluster_cloud)
    stat_outlier_filter_pub.publish(ros_outlier_filtered_cloud)



# Exercise-3 TODOs:

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

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
       pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    ## Initialize variables (was TODO)
    labels = []
    centroids = [] # to be list of tuples (x, y, z)
    ## Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format (was TODO)
    dict_list = []

    ## Get/Read parameters (was TODO)
    # Since the header of the pick_list_* file is object_list,
    # that is the parameter name under which is loaded
    object_list_param = rospy.get_param('/object_list') # parameter name
    # Get/Read parameters of dropbox positions
    dropbox_obj_param = rospy.get_param('/dropbox') # header of the dropbox.yaml file is dropbox

    # TODO: Rotate PR2 in place to capture side tables for the collision map

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

        ## Pick pose, calculated pose of recognized object's centroid (was TODO)
        # Initialize an empty pose message
        pick_pose = Pose()

        # WARNING: ROS messages expect native Python data types but having computed centroids as above your list centroids will be of type numpy.float64
        # To recast to native Python float type you can use np.asscalar()

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


        #try:
        #    pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

        #    ## Insert your message variables to be sent as a service request (was TODO)
        #    resp = pick_place_routine(test_scene_num, msg_object_name, which_arm, pick_pose, place_pose)
        #    print ("Response: ",resp.success)

        #except rospy.ServiceException, e:
        #    print "Service call failed: %s"%e

    ## end of for loop ##    

    ## Output your request parameters into output yaml file (was TODO)
    # yaml filenames: output_1.yaml, output_2.yaml, and output_3.yaml
    yaml_filename = 'output_'+str(test_scene)+'.yaml'
    send_to_yaml(yaml_filename, dict_list) # list of dictionaries
    print "Saved output yaml file."



if __name__ == '__main__':

    ## ROS node initialization (was TODO)
    rospy.init_node('object_recognition', anonymous=True)

    ## Create Subscribers (was TODO)
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    ## Create Publishers 
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    # Cloud containing all clusters (objects), each with unique color:
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    # Here you need to create two new publishers (was TODO)
    # Call them object_markers_pub and detected_objects_pub
    # Have them publish to "/object_markers" and "/detected_objects" with 
    # Message Types "Marker" and "DetectedObjectsArray" , respectively
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
    stat_outlier_filter_pub = rospy.Publisher("/pcl_stat_outlier_filter", PointCloud2, queue_size=1)

    ## Load Model From disk (was TODO)
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    ## Spin while node is not shutdown (was TODO)
    while not rospy.is_shutdown():
        rospy.spin()
