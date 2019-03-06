#!/usr/bin/env bash

# This scripts builds the Python module in setup.py, and then installs or 
# upgrades to the latest version using Pip (at user level).
#
# The first command line argument should be the Python executable.
#
# Example usage:
#    Install-Python.sh /usr/bin/python3
#    Install-Python.sh $(which python3.7)

python_bin=$1
python_version_dot=$(${python_bin} -c 'import sys; print(sys.version_info[0], sys.version_info[1], sep=".")')
python_version_no_dot=$(${python_bin} -c 'import sys; print(sys.version_info[0], sys.version_info[1], sep="")')
echo "Using Python $python_version_dot (${python_bin})"

# Build the Wheel package
$python_bin setup.py bdist_wheel
# List all Wheels compatible with this Python version
wheels=($(ls dist/py_drone_module-*-cp${python_version_no_dot}-cp${python_version_no_dot}m-linux_x86_64.whl))
# Sort them by version number
wheels=($(sort --version-sort <<< ${wheels[*]}))
echo "${wheels[*]}"
# Select the last element (newest version)
wheel=${wheels[-1]}
# Install the package
$python_bin -m pip install --user --upgrade "$wheel"