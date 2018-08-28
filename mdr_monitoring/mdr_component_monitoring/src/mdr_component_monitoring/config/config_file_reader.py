from __future__ import print_function
from os.path import join
import yaml
from mdr_component_monitoring.config.config_params import ComponentMonitorConfig, MonitorModeConfig, \
                                                          FunctionalMappingConfig, OutputConfig

'''An interface for reading component monitor configuration files

@author Alex Mitrevski, Santosh Thoduka
@contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
'''
class ComponentMonitorConfigFileReader(object):
    '''Loads the configuration parameters of a component monitor from the given YAML file

    Keyword arguments:
    @param root_dir component monitor configuration file directory
    @param config_file_name absolute path of a config file

    '''
    @staticmethod
    def load(root_dir, config_file_name):
        params = ComponentMonitorConfig()

        file_path = join(root_dir, config_file_name)
        root = ComponentMonitorConfigFileReader.__read_yaml_file(file_path)
        if 'name' in root.keys():
            params.name = root['name']
        else:
            print('name not specified')
            return ComponentMonitorConfig()

        if 'modes' in root.keys():
            for mode_config_file in root['modes']:
                mode_config = ComponentMonitorConfigFileReader.__load_mode_config(root_dir,
                                                                                  mode_config_file)
                params.modes.append(mode_config)
        else:
            print('modes not specified')
            return ComponentMonitorConfig()

        return params

    '''Loads the configuration parameters of a component monitor mode from the given YAML file

    Keyword arguments:
    @param root_dir component monitor configuration file directory
    @param config_file_name absolute path of a config file

    '''
    @staticmethod
    def __load_mode_config(root_dir, config_file_name):
        params = MonitorModeConfig()

        file_path = join(root_dir, config_file_name)
        root = ComponentMonitorConfigFileReader.__read_yaml_file(file_path)

        if 'name' in root.keys():
            params.name = root['name']
        else:
            print('mode_config: name not specified')
            return MonitorModeConfig()

        if 'mappings' in root.keys():
            for mapping in root['mappings']:
                mapping_node = mapping['mapping']

                fn_mapping_params = FunctionalMappingConfig()
                fn_mapping_params.inputs = mapping_node['inputs']
                for output in mapping_node['outputs']:
                    output_node = output['output']

                    output_params = OutputConfig()
                    output_params.name = output_node['name']
                    output_params.obtained_value_type = output_node['type']
                    if 'expected' in output.keys():
                        output_params.expected_value = output_node['expected']
                    fn_mapping_params.outputs.append(output_params)
                params.mappings.append(fn_mapping_params)
        else:
            print('mode_config: mappings not specified')
            return MonitorModeConfig()

        if 'arguments' in root.keys():
            for argument in root['arguments']:
                argument_node = argument['arg']
                arg_name = argument_node['name']
                arg_value = argument_node['value']
                params.arguments[arg_name] = arg_value

        return params

    @staticmethod
    def __read_yaml_file(file_name):
        file_handle = open(file_name, 'r')
        data = yaml.load(file_handle)
        file_handle.close()
        return data
