from setuptools import setup

setup(
    name="youi",
    description='editable, live python UI',
    author='Daniel Dugas',
    version='0.0.1',
    packages=["youi", "common_srvs"],
#     ext_modules=cythonize("something/something.pyx", annotate=True),
    python_requires='>=3.6',
    install_requires=[
        'numpy',
        'matplotlib',
        'pyqt5'
    ],
#     include_dirs=[numpy.get_include()],
#     package_data={'navrep': ['maps/*']},
)
