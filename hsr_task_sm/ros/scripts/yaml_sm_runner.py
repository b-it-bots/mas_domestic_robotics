#!/usr/bin/env python3
"""
YAML State Machine Runner

Run state machines defined in YAML configuration files.

Usage:
    rosrun hsr_task_sm yaml_sm_runner.py _config:=hri_challenge.yaml
    rosrun hsr_task_sm yaml_sm_runner.py _config:=/full/path/to/config.yaml

Parameters:
    ~config (str): Path to YAML config file (required)
    ~validate_only (bool): Only validate, don't run (default: false)
    ~introspection (bool): Enable SMACH viewer (default: true)
"""

import os
import sys
import rospy
import smach_ros

from hsr_task_sm.yaml_sm_loader import YAMLStateMachineLoader, validate_config


def main():
    rospy.init_node('yaml_sm_runner')
    
    # Get config path
    config_path = rospy.get_param('~config', '')
    
    if not config_path:
        rospy.logerr('Missing required parameter: ~config')
        rospy.logerr('Usage: rosrun hsr_task_sm yaml_sm_runner.py _config:=<config.yaml>')
        sys.exit(1)
    
    # Resolve relative paths
    if not os.path.isabs(config_path):
        import rospkg
        try:
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('hsr_task_sm')
            config_path = os.path.join(pkg_path, 'config', 'challenges', config_path)
        except Exception:
            rospy.logerr('Cannot resolve path: %s', config_path)
            sys.exit(1)
    
    if not os.path.exists(config_path):
        rospy.logerr('Config file not found: %s', config_path)
        sys.exit(1)
    
    # Validate first
    validate_only = rospy.get_param('~validate_only', False)
    
    rospy.loginfo('Validating config: %s', config_path)
    errors = validate_config(config_path)
    
    if errors:
        rospy.logerr('Validation failed:')
        for error in errors:
            rospy.logerr('  - %s', error)
        sys.exit(1)
    
    rospy.loginfo('Config validation passed!')
    
    if validate_only:
        rospy.loginfo('Validation only mode, exiting.')
        sys.exit(0)
    
    # Load and run
    loader = YAMLStateMachineLoader()
    sm = loader.load(config_path)
    
    # Optional introspection server
    introspection = rospy.get_param('~introspection', True)
    sis = None
    
    if introspection:
        config_name = os.path.splitext(os.path.basename(config_path))[0]
        sis = smach_ros.IntrospectionServer(config_name, sm, f'/{config_name.upper()}')
        sis.start()
        rospy.loginfo('SMACH viewer available. Run: rosrun smach_viewer smach_viewer.py')
    
    # Execute
    rospy.loginfo('Starting state machine...')
    outcome = sm.execute()
    rospy.loginfo('State machine finished with outcome: %s', outcome)
    
    if sis:
        sis.stop()
    
    return outcome


if __name__ == '__main__':
    try:
        outcome = main()
        sys.exit(0 if outcome in ['SUCCEEDED', 'DONE'] else 1)
    except rospy.ROSInterruptException:
        rospy.loginfo('Interrupted')
        sys.exit(0)
    except Exception as e:
        rospy.logerr('Error: %s', e)
        import traceback
        traceback.print_exc()
        sys.exit(1)
