from setuptools import find_packages, setup


package_name = "nav2_ready"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Nav2 Ready contributors",
    maintainer_email="89831723+Issey-Kubota@users.noreply.github.com",
    description="Preflight diagnostics for connecting a custom robot to Nav2.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["check = nav2_ready.cli:main"]},
)
