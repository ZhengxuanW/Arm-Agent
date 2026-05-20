from setuptools import setup

package_name = "moveit_probe"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="local",
    maintainer_email="local@example.invalid",
    description="Small MoveIt sandbox probes.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "wait_for_move_group = moveit_probe.wait_for_move_group:main",
        ],
    },
)
