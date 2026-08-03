"""Helpers shared by the launch files for reading the YAML configuration files.

Kept separate so view_camera_launch.py and view_rtsp_camera_launch.py behave the same
way: the config file holds every parameter, and a command line argument only overrides
it when the user actually passes one.
"""

import yaml

from launch.substitutions import LaunchConfiguration

# Section that applies to every node in a ROS 2 parameter file.
WILDCARD = '/**'


def load_node_parameters(config_file, node_name):
    """Return the parameters a node gets from a standard ROS 2 parameter file.

    The wildcard section is applied first so that a node-specific section wins where
    the two define the same name.
    """
    with open(config_file, 'r') as handle:
        document = yaml.safe_load(handle) or {}

    parameters = {}
    for section in (WILDCARD, node_name, '/' + node_name):
        values = document.get(section)
        if isinstance(values, dict):
            parameters.update(values.get('ros__parameters', {}))
    return parameters


def cast_like(reference, text):
    """Convert a command line string to the type of the value it replaces.

    Launch hands every argument over as a string. Passing a string where the node
    declared an int or a bool makes the node throw a type error on startup, so the
    value already in the config file is used to pick the target type.
    """
    if isinstance(reference, bool):
        return text.strip().lower() in ('1', 'true', 'yes', 'on')
    if isinstance(reference, int):
        return int(text)
    if isinstance(reference, float):
        return float(text)
    return text


def override_parameters(parameters, overrides, context):
    """Apply the non-empty command line arguments on top of the config file values.

    parameters: {node name: {parameter name: value}}, modified in place.
    overrides:  {launch argument: [(node name, parameter name), ...]}
    """
    for argument, targets in overrides.items():
        text = LaunchConfiguration(argument).perform(context)
        if text == '':
            continue
        for node_name, parameter in targets:
            node_parameters = parameters.get(node_name)
            if node_parameters is None:
                continue
            reference = node_parameters.get(parameter, text)
            node_parameters[parameter] = cast_like(reference, text)
