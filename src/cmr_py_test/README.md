# cmr_py_test

@brief Integration and node-level unit tests in Python.

Add your Python tests in a file ending with `_test.py` in the `test` directory
of this package. `from test_utils import*` at the top of your test file. Tests
may be seperate methods [decorated](https://book.pythontips.com/en/latest/decorators.html) 
with `@cmr_py_test` or as methods of a class that inherits from `CMRTestFixture`.
In both cases, the function or method name should begin with `test_` so that Pytest
may find the file.

Tests will run via the test script in `scripts/test.sh` which finds all test files
in the `test` directory of this package and runs them under `pytest`. You may also run
the tests manually using `pytest`. Some helpful command line arguments of `pytest`
are:
* `-x` - Stop at first failure
* `-v` - Verbose
* `--tb=short`, `--tb=long` - set the traceback length on error
* `-s` - Do not capture stdout
* `-k <filter>` - Run only the tests with pass the `<filter>`

To launch nodes and or run launch files before your See the `test_utils` 
module for more information.

Integration tests will be run on the CI using the `scripts/test_wd.sh` script. This
is effectively the same thing as the `test` task (running `scripts/test.sh`) in VSCode.
This can be found in the `.github/build-and-test.yaml` file.
