"""
Launch specific utilities.
"""

import yaml
import xacro
from os import path
from typing import IO

from ament_index_python.packages import get_package_share_directory


def load_file(
    package_name: str, file_path: str, mappings: dict[str:str] = {"": ""}
) -> IO[str]:
    """
    Reads a file and returns its contents.
    Differentiates between xacro, yaml and other files (xml)
    based on their suffix.

    Args:
        package_name (str): the name of the package
        file_path (str): the filepath of the file to be read

    Returns:
        Contents of file
    """

    # create absolute file path
    package_path = get_package_share_directory(package_name)
    absolute_file_path = path.join(package_path, file_path)
    # get file suffix
    file_type = path.splitext(file_path)[1]
    try:
        if file_type == ".xacro":
            # xacro requires processing
            return xacro.process_file(
                absolute_file_path, mappings=mappings
            ).toprettyxml()
        # open file
        with open(absolute_file_path, "r") as file:
            if file_type == ".yaml":
                # yaml requires safe load
                return yaml.safe_load(file)
            else:
                # everything else will be returned directly
                return file.read()
    except OSError as e:
        print(
            "\n\nSomething went wrong reading the file "
            + absolute_file_path
            + "\n Error message: "
            + str(e)
            + "\n\n"
        )
        return None
