#!/usr/bin/env python3

from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="ros2top",
    version="0.1.0",
    author="Ahmed Radwan",
    author_email="ahmed.ali.radwan94@gmail.com",
    description="A real-time monitor for ROS2 nodes showing CPU, RAM, and GPU usage",
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages=find_packages(),
    
    # Install Python package data
    package_data={
        'ros2top': ['*.py'],
    },
    
    # Install C++ headers and CMake files
    data_files=[
        ('include/ros2top', ['include/ros2top/ros2top.hpp']),
        ('share/ros2top/cmake', ['cmake/ros2topConfig.cmake']),
    ],
    
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Topic :: System :: Monitoring",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
    python_requires=">=3.8",
    install_requires=[
        "psutil>=5.8.0",
        "pynvml>=11.0.0",
    ],
    entry_points={
        "console_scripts": [
            "ros2top=ros2top.main:main",
        ],
    },
    keywords="ros2, monitoring, gpu, cpu, memory, nodes",
    project_urls={
        "Bug Reports": "https://github.com/AhmedARadwan/ros2top/issues",
        "Source": "https://github.com/AhmedARadwan/ros2top",
    },
)
