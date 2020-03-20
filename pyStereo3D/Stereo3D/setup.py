import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="Stereo3D",
    version="0.0.1.13.6",
    author="Ben Knight",
    author_email="bknight@i3drobotics.com",
    description="Generating 3D data from stereo images.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/i3drobotics/Stereo3D",
    packages=setuptools.find_packages(),
    package_dir={'Stereo3D':'Stereo3D'},
    package_data={'Stereo3D':['StereoCapture/coppeliasim/data/*.dll']},
    install_requires=[
        'numpy','opencv-python','pymsgbox','pypylon', 'pyntcloud'
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.5',
)