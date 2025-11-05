"""
Configuration loader for YAML files.

This module provides utilities to load YAML configuration files
and convert them to LeRobot configuration objects.
"""

from pathlib import Path
from typing import Any, Dict, Union
import yaml


def load_config(config_path: Union[str, Path]) -> Any:
    """
    Load configuration from YAML file.

    This function loads a YAML file and returns it as a dictionary.
    The dictionary can then be used to create LeRobot config objects.

    Args:
        config_path: Path to YAML configuration file

    Returns:
        Configuration dictionary or LeRobot config object

    Raises:
        FileNotFoundError: If config file doesn't exist
        yaml.YAMLError: If YAML is invalid
    """
    config_path = Path(config_path)

    if not config_path.exists():
        raise FileNotFoundError(f"Configuration file not found: {config_path}")

    try:
        with open(config_path, "r") as f:
            config_dict = yaml.safe_load(f)

        return config_dict

    except yaml.YAMLError as e:
        raise ValueError(f"Invalid YAML in {config_path}: {e}")


def save_config(config: Dict[str, Any], config_path: Union[str, Path]):
    """
    Save configuration to YAML file.

    Args:
        config: Configuration dictionary to save
        config_path: Path to save YAML file

    Raises:
        yaml.YAMLError: If config cannot be serialized
    """
    config_path = Path(config_path)

    # Create parent directories if needed
    config_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        with open(config_path, "w") as f:
            yaml.safe_dump(config, f, default_flow_style=False, sort_keys=False)

        print(f"✅ Configuration saved to: {config_path}")

    except yaml.YAMLError as e:
        raise ValueError(f"Failed to save config to {config_path}: {e}")


def dict_to_lerobot_config(config_dict: Dict[str, Any], config_class: type):
    """
    Convert dictionary to LeRobot config object.

    Args:
        config_dict: Configuration dictionary
        config_class: LeRobot config class (e.g., SO100FollowerConfig)

    Returns:
        Instance of config_class
    """
    try:
        return config_class(**config_dict)
    except Exception as e:
        raise ValueError(
            f"Failed to create {config_class.__name__} from dict: {e}\n"
            f"Config dict: {config_dict}"
        )


def merge_configs(base_config: Dict, override_config: Dict) -> Dict:
    """
    Merge two configuration dictionaries.

    Override config takes precedence over base config.

    Args:
        base_config: Base configuration
        override_config: Configuration to override base

    Returns:
        Merged configuration dictionary
    """
    merged = base_config.copy()

    for key, value in override_config.items():
        if (
            key in merged
            and isinstance(merged[key], dict)
            and isinstance(value, dict)
        ):
            # Recursively merge nested dicts
            merged[key] = merge_configs(merged[key], value)
        else:
            # Override value
            merged[key] = value

    return merged


def load_robot_config(config_path: Union[str, Path]):
    """
    Load robot configuration and convert to LeRobot format.

    Args:
        config_path: Path to robot configuration YAML

    Returns:
        Robot configuration (dictionary format compatible with LeRobot)
    """
    config = load_config(config_path)

    # Transform to LeRobot format if needed
    # This is where we'd convert our YAML format to LeRobot's expected format

    return config


def load_camera_config(config_path: Union[str, Path]):
    """
    Load camera configuration.

    Args:
        config_path: Path to camera configuration YAML

    Returns:
        Camera configuration dictionary
    """
    return load_config(config_path)


def load_training_config(config_path: Union[str, Path]):
    """
    Load training configuration.

    Args:
        config_path: Path to training configuration YAML

    Returns:
        Training configuration dictionary
    """
    return load_config(config_path)
