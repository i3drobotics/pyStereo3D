import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="TemplatePackage",
    version="0.0.1",
    author="Ben Knight",
    author_email="bknight@i3drobotics.com",
    description="Template package",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/i3drobotics/TemplatePackage",
    packages=setuptools.find_packages(),
    install_requires=[
        'numpy','opencv-python','stereocapture'
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)