from setuptools import setup, find_packages

setup(
    name='robotics-monitor-server',
    version='1.0.0',
    packages=find_packages(),
    install_requires=[
    ],
    entry_points={
        'console_scripts': [
            'robotics-monitor-server=main:main',  # Replace with your app's entry point
        ],
    },
    author='Your Name',
    author_email='your.email@example.com',
    description='Your Flask Application Description',
    license='MIT',
    keywords='flask gunicorn web application',
    url='https://github.com/yourusername/your-flask-app',
)