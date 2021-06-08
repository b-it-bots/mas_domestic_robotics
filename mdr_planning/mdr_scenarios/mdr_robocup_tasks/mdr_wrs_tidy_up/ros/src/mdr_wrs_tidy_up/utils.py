import dynamic_reconfigure.client
import rospy
from mas_tools.ros_utils import get_package_path
from mas_tools.file_utils import load_yaml_file

def load_default_object_detection_params(filename=None):
    if filename:
        params_file = get_package_path('mdr_cloud_object_detection', 'ros/config', filename)
    else:
        params_file = get_package_path('mdr_cloud_object_detection', 'ros/config', 'object_detection_default_configs.yaml')
    params = load_yaml_file(params_file)
    if params:
        return params
    else:
        rospy.logerr("Unable to load default object_detection_params from file {0}".format(params_file))
        return None

def load_custom_object_detection_params(filename=None):
    if filename:
        params_file = get_package_path('mdr_wrs_tidy_up', 'config', filename)
    else:
        params_file = get_package_path('mdr_wrs_tidy_up', 'config', 'object_detection_params.yaml')
    params = load_yaml_file(params_file)
    if params:
        return params
    else:
        rospy.logerr("Unable to load custom object_detection_params from file {0}".format(params_file))
        return None

def reconfigure_object_detection_params(updates, node_name = "/mdr_cloud_object_detection/cloud_object_detection"):
    try:
        client = dynamic_reconfigure.client.Client(node_name, timeout=1.5)
    except Exception as exc:
        rospy.logerr("Service {0} does not exist: {1}".format(node_name + '/set_parameters', str(exc)))
        return False

    client.update_configuration(updates)

def update_object_detection_params(surface, custom_config_file=None, default_config_file=None):
    default_params = load_default_object_detection_params(default_config_file)
    if not default_params:
        rospy.logerr("Unable to load default object_detection_params")

    custom_params = load_custom_object_detection_params(custom_config_file)
    if custom_params:
        surface_params = custom_params[surface]
        if surface_params:
            updated_params = default_params.copy()
            for key in surface_params:
                if key in updated_params:
                    updated_params[key] = surface_params[key]
                else:
                    rospy.logerr("Unable to set the cloud object detection param {0} for surface {1}. Invalid param name '{0}'".format(key, surface))
            reconfigure_object_detection_params(updated_params)
        else:
            rospy.logerr("Unable to load custom/defa object_detection_params for surface {0}".format(surface))
