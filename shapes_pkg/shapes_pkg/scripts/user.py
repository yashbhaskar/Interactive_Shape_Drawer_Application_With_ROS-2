#!/usr/bin/env python3

import subprocess

def main():
    print("Which shape do you want to draw? (square/star/triangle):")
    shape = input().strip().lower()

    if shape not in ['square', 'star', 'triangle']:
        print("Invalid shape. Please choose from 'square', 'star', or 'triangle'.")
        return

    if shape == 'square':
        node_name = 'Square'
        executable = 'square.py'
    elif shape == 'star':
        node_name = 'Star'
        executable = 'star.py'
    elif shape == 'triangle':
        node_name = 'Triangle'
        executable = 'triangle.py'

    cmd = [
        'ros2', 'run', 'shapes_pkg', executable
    ]

    subprocess.run(cmd)

if __name__ == '__main__':
    main()
