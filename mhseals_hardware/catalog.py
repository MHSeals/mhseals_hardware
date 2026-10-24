"""Explicit installed hardware descriptor binding; no ROS or output lifecycle."""
from mhseals_hardware.resources import config_resource

from mhseals_config.catalog import DescriptorBundle
from mhseals_hardware.configuration import validate_config
from mhseals_hardware.startup_config import compile_inputs, node_startup_inputs
from mhseals_hardware.control_validation import controller_section


# Domain functions take their original full closures, never individual leaf values.
REGISTRY = {
    'hardware.validate_config': validate_config,
    'hardware.compile_inputs': compile_inputs,
    'hardware.node_startup_inputs': node_startup_inputs,
    'hardware.controller_section': controller_section,
}


def load_catalog(*, consumer_versions=None):
    """Read only package resources. Missing installed consumer builds stay blocked."""
    return DescriptorBundle.read_yaml(
        config_resource('catalog.yaml').read_text(),
        {'hardware-default': config_resource('default.yaml').read_bytes()},
        REGISTRY, consumer_versions or {})
