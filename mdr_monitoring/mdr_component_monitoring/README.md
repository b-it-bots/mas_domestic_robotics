# mdr_component_monitoring

## Procedure for adding new monitors

1. Create a monitor configuration file `monitor_config/<type>/<component-name>.yaml`, where `<type>` is either `hardware` or `software` depending on the type of component, and describe the monitoring modes as described (here)[https://github.com/ropod-project/component-monitoring]
2. Create a directory `monitor_config/<type>/<component-name>` and add mode configuration files there
3. Create a directory `src/mdr_component_monitoring/monitors/<type>/<component-name>` and implement the mode monitors in separate scripts; the monitors should inherit from the `MonitorBase` class defined in `src/mdr_component_monitoring/monitor_base.py`
4. Add constants with the monitor mode names in `HardwareMonitorNames` or `SoftwareMonitorNames` depending on the type of component; the values of the constants should match the names of the monitor mode names assigned in the mode configuration files
5. Update `get_hardware_monitor` or `get_software_monitor` in `src/mdr_component_monitoring/monitor_factory.py` so that they generate objects of the newly implemented monitors
