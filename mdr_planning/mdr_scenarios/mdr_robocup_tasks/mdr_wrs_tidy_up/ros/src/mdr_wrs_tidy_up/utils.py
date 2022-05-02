import dynamic_reconfigure.client
import rospy
from mas_tools.ros_utils import get_package_path
from mas_tools.file_utils import load_yaml_file

def load_obstacle_detection_params(filename=None):
    if filename:
        params_file = get_package_path('mdr_wrs_tidy_up', 'config', filename)
    else:
        params_file = get_package_path('mdr_wrs_tidy_up', 'config', 'object_detection_params.yaml')
    params = load_yaml_file(params_file)
    if params:
        return params
    else:
        rospy.logerr("Unable to load object_detection_params from file {0}".format(params_file))
        return None

def reconfigure_object_detection_params(z_min, z_max, node_name = "/mdr_cloud_object_detection/cloud_object_detection"):
    try:
        client = dynamic_reconfigure.client.Client(node_name, timeout=1.5)
    except Exception as exc:
        rospy.logerr("Service {0} does not exist: {1}".format(node_name + '/set_parameters', str(exc)))
        return False

    updates = {"cropbox_limit_min_z":  z_min,
               "cropbox_limit_max_z":  z_max}
    client.update_configuration(updates)

def update_object_detection_params(surface, config_file=None):
    params = load_obstacle_detection_params(config_file)
    if params:
        config = params[surface]
        if config:
            reconfigure_object_detection_params(z_min=config["z_min"], z_max=config["z_max"])
        else:
            rospy.logerr("Unable to load object_detection_params for surface {0}".format(surface))
