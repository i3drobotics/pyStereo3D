import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="Stereo3D",
    version="0.0.1.10",
    author="Ben Knight",
    author_email="bknight@i3drobotics.com",
    description="Generating 3D data from stereo images.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/i3drobotics/Stereo3D",
    packages=setuptools.find_packages(),
    install_requires=[
        'stereocapture'
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)