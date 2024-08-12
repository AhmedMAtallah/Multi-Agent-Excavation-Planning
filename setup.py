from setuptools import setup, find_packages

setup(
    name="multi_agent_digging_planning",
    version="0.1",
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    install_requires=[
        "matplotlib",
        "numpy",
    ],
)
