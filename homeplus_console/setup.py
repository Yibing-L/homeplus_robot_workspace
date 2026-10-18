from glob import glob

from setuptools import setup

package_name = "homeplus_console"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    package_data={package_name: ["static/*", "static/assets/*"]},
    include_package_data=True,
    install_requires=["setuptools", "fastapi>=0.110", "uvicorn[standard]", "pydantic>=2", "PyYAML", "Pillow", "numpy"],
    zip_safe=False,
    entry_points={"console_scripts": ["console = homeplus_console.main:main"]},
)
