import setuptools

with open("../README.md", "r") as fh:
    long_description = fh.read()

with open("../version.txt", "r") as fh:
    version = fh.read()

setuptools.setup(
    name="stereo3d",
    version=version,
    author="Ben Knight",
    author_email="bknight@i3drobotics.com",
    description="Generating 3D data from stereo images.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/i3drobotics/Stereo3D",
    packages=setuptools.find_packages(),
    package_dir={'stereo3d':'stereo3d'},
    package_data={'stereo3d':[
        'StereoCapture/coppeliasim/data/MacOS/remoteApi.dylib',
        'StereoCapture/coppeliasim/data/Ubuntu16_04/remoteApi.so',
        'StereoCapture/coppeliasim/data/Ubuntu18_04/remoteApi.so',
        'StereoCapture/coppeliasim/data/Windows/remoteApi.dll',
    ]},
    install_requires=[
        'numpy; python_version == "3.5"',
        'numpy==1.19.3; python_version > "3.5"',
        'opencv-python','pymsgbox','pypylon', 'pyntcloud'
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)