from setuptools import find_packages, setup

VERSION = "0.0.1"
DESCRIPTION = ""
LONG_DESCRIPTION = """
""" 

setup(
    name="project3",
    packages=find_packages(),
    version=VERSION, 
    description="project3",
    author="MJ",
    author_email="temp",
    keywords="project3", 
    license="MIT",
    long_description=LONG_DESCRIPTION,
    long_description_content_type="text/markdown",
    python_requires=">=3.6",
    # PyPI package information.
    classifiers=[
        "Programming Language :: Python :: 3",
    ],

)
# python setup.py sdist bdist_wheel
# twine upload --repository-url https://test.pypi.org/legacy/ dist/*
# twine upload dist/*
