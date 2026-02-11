from glob import glob
from setuptools import setup

package_name = "caatinga_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/models", glob("models/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="joaodemoura",
    maintainer_email="joao@example.com",
    description="Pipeline de visao computacional para rastreabilidade inteligente",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_source_node = caatinga_vision.camera_source_node:main",
            "yolo_inference_node = caatinga_vision.yolo_inference_node:main",
            "infestation_analytics_node = caatinga_vision.infestation_analytics_node:main",
            "photo_capture_node = caatinga_vision.photo_capture_node:main",
        ],
    },
)
