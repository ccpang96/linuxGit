There are 2 kind of tests for gtrack: Usecase test and functional unit test.
Only the functional unit tests are released to customers presently. The makefile
does not list the usecase make options. But they can be built using the following
command under git (will not work from released package).

For xwr16xx:
-----------
# To clean the lib and usecase tests
gmake libClean usecaseDssTestClean usecaseMssTestClean

# To build the lib and usecase tests
gmake lib usecaseDssTest usecaseMssTest

For xwr14xx:
-----------
# To clean the lib and usecase tests
gmake libClean usecaseMssTestClean

# To build the lib and usecase tests
gmake lib usecaseMssTest
