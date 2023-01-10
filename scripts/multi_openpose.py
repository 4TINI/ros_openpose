#!/usr/bin/env python3

import roslaunch
import rospy
import rospkg
from sensor_msgs.msg import CameraInfo
from roslaunch.parent import ROSLaunchParent
import math

class ROSLauncher(object):
    def __init__(self, rospackage_name, launch_path, arguments=None):
        self._rospackage_name = rospackage_name
        self.rospack = rospkg.RosPack()
        self._path_launch_file_name = launch_path

        # Check Package Exists
        try:
            pkg_path = self.rospack.get_path(rospackage_name)
        except rospkg.common.ResourceNotFound:
            rospy.logwarn("Package NOT FOUND...")
    
        # If the package was found then we launch
        if pkg_path:
            if(arguments is not None):
                roslaunch_file = [(self._path_launch_file_name, arguments)]    
            else:
                roslaunch_file = [self._path_launch_file_name]                                      
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = ROSLaunchParent(self.uuid, 
                                         roslaunch_file)        
        else:
            assert False, ("No Package Path was found for ROS package ==>" + 
                          str(self._rospackage_name))

    def start(self):
        self.launch.start()

    def shutdown(self):
        self.launch.shutdown()
    
if __name__ == '__main__':

    rospy.init_node('multi_openpose')

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()

    # get the file path for rospy_tutorials
    pack_path = rospack.get_path('open_vico_openpose')

    cameras_list = rospy.get_param('~cameras_list', '[0]')
    res = rospy.get_param('~res', 368)
    number_people_max = rospy.get_param('~number_people_max', 1)
    headless_mode = rospy.get_param('~headless_mode', False)
    num_gpus = rospy.get_param('~num_gpus', 2)

    # cameras_list = [[int(x.strip(' ')) for x in ss.lstrip(' [,').split(', ')] for ss in cameras_list.rstrip(']').split(']')]
    cameras_list = [int(item) for sublist in cameras_list for item in sublist if item.isdigit()]
    
    gpu_ids = []
    for i in range(num_gpus):
        if i == num_gpus-1:
            gpu_ids.extend([i] * (len(cameras_list)//num_gpus))
        else:
            gpu_ids.extend([i] * math.ceil(len(cameras_list)/num_gpus))

    for count, value in enumerate(cameras_list):
        rospy.loginfo("Waiting for topic: %s to be published", '/camera'+str(value)+'/rgb/camera_info')
        rospy.wait_for_message('/camera'+str(value)+'/rgb/camera_info', CameraInfo)

        launch_args =[]
        launch_args.append(pack_path+'/launch/openpose.launch')

        launch_args.append('res:='+str(res))
        launch_args.append('number_people_max:='+str(number_people_max))
        launch_args.append('headless_mode:='+str(headless_mode))
        launch_args.append('gpu_id:='+str(gpu_ids[count]))
        launch_args.append('camera_number:='+str(value))
        launch_args.append('color_topic:=/camera'+str(value)+'/rgb/image_raw')
        launch_args.append('pub_topic:=/camera'+str(value)+'/frame')
        openpose_launch = ROSLauncher('open_vico_openpose',
                                        rospack.get_path('open_vico_openpose')+'/launch/openpose.launch',
                                        launch_args)
                                        
        openpose_launch.start()

        
    while not rospy.is_shutdown():        
        rospy.spin()