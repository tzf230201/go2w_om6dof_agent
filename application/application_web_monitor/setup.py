from glob import glob

from setuptools import setup


package_name = "application_web_monitor"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/systemd", glob("systemd/*.service")),
        ("share/" + package_name + "/sudoers", glob("sudoers/*")),
        ("share/" + package_name + "/skills", glob("skills/*.md")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="unitree",
    maintainer_email="biancanobelia@gmail.com",
    description="Network web monitor for the Go2W and OM6DOF applications.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "web_monitor = application_web_monitor.web_monitor:main",
            "unitree_camera_relay = "
            "application_web_monitor.unitree_camera_relay:main",
            "unitree_audio_bridge = "
            "application_web_monitor.unitree_audio_bridge:main",
            "unitree_stt = application_web_monitor.unitree_stt:main",
            "dji_audio_bridge = "
            "application_web_monitor.dji_audio_bridge:main",
            "audio_launcher = "
            "application_web_monitor.audio_launcher:main",
            "stt_llm_bridge = "
            "application_web_monitor.stt_llm_bridge:main",
            "kokoro_tts = application_web_monitor.kokoro_tts:main",
        ],
    },
)
