from setuptools import setup

package_name = "scara_trajectory"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SARIN Chandevid",
    maintainer_email="sarinchandevid2019@gmail.com",
    description="SCARA trajectory generation, digital twin and visualisation nodes",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ik_to_rviz_controller = scara_trajectory.ik_to_rviz_controller:main",
            "color_cube_visualizer = scara_trajectory.color_cube_visualizer:main",
            "home_position         = scara_trajectory.home_position:main",
            "pick_place_demo       = scara_trajectory.pick_place_demo:main",
            "square_demo           = scara_trajectory.square_demo:main",
            "recorder              = scara_trajectory.recorder:main",
            "twin_recorder         = scara_trajectory.twin_recorder:main",
        ],
    },
)
