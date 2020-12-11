import dynamic_reconfigure.client
import rospy


def reconfigure_object_detection_params(z_min, z_max, node_name = "/mas_perception/cloud_obstacle_detection"):
    try:
        client = dynamic_reconfigure.client.Client(node_name, timeout=1.5)
    except Exception as e:
        rospy.logerr("Service {0} does not exist".format(node_name + '/set_parameters'))
        return False

    updates = {"passthrough_limit_min_z":  z_min,
               "passthrough_limit_max_z":  z_max}
    client.update_configuration(updates)
