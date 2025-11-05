"""
Logging utilities
"""

import logging
import sys


def get_logger(name):
    """
    Get logger

    Args:
        name: Logger name

    Returns:
        logging.Logger
    """
    return logging.getLogger(name)


def setup_logging(level=logging.INFO):
    """
    Logging configuration

    Args:
        level: Log level (Default: INFO)
    """
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(sys.stdout)
        ]
    )
