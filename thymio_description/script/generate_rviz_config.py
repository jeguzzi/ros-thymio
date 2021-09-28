#!/usr/bin/env python

import sys
import os


def rviz(launch_folder: str, name: str) -> None:
    source_path = os.path.join(launch_folder, 'template.rviz')
    with open(source_path, 'r') as f:
        source = f.read()
    if name and name[0] != "/":
        ns = f"/{name}"
    else:
        ns = name
    dest = source.replace('/thymio', ns).replace('thymio', name)
    dest_path = os.path.join(launch_folder, 'thymio.rviz')
    with open(dest_path, 'w') as f:
        f.write(dest)


if __name__ == '__main__':
    rviz(sys.argv[1], sys.argv[2])