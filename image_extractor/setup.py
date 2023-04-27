from setuptools import setup

package_name = 'image_extractor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
    ],
    description=(
        'A package to extract images from a ROS2 bag file.'
    ),
    license='MIT License',
    entry_points={
        'console_scripts': [
            'image_extractor = image_extractor.image_extractor:main',
            'depth_extractor = image_extractor.depth_extractor:main',

        ],
    },
)
