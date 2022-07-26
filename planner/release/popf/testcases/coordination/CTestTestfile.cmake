# CMake generated Testfile for 
# Source directory: /home/iannuz/popf-tif/planner/src/popf/testcases/coordination
# Build directory: /home/iannuz/popf-tif/planner/release/popf/testcases/coordination
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(driverlogshift1-can-be-solved "/home/iannuz/popf-tif/planner/release/popf/popf2" "-I" "-D" "-T" "-v1" "/home/iannuz/popf-tif/planner/src/popf/testcases/coordination/driverlogshift-domain.pddl" "/home/iannuz/popf-tif/planner/src/popf/testcases/coordination/driverlogshift-problem-01.pddl")
set_tests_properties(driverlogshift1-can-be-solved PROPERTIES  _BACKTRACE_TRIPLES "/home/iannuz/popf-tif/planner/src/popf/testcases/coordination/CMakeLists.txt;1;add_test;/home/iannuz/popf-tif/planner/src/popf/testcases/coordination/CMakeLists.txt;0;")
add_test(driverlogshift2-plan-could-be-found "/home/iannuz/popf-tif/planner/release/popf/popf2" "-r" "-s" "-I" "-D" "-v1" "-T" "/home/iannuz/popf-tif/planner/src/popf/testcases/coordination/driverlogshift-domain.pddl" "/home/iannuz/popf-tif/planner/src/popf/testcases/coordination/driverlogshift-problem-02.pddl" "/home/iannuz/popf-tif/planner/src/popf/testcases/coordination/driverlogshift-plan-02")
set_tests_properties(driverlogshift2-plan-could-be-found PROPERTIES  _BACKTRACE_TRIPLES "/home/iannuz/popf-tif/planner/src/popf/testcases/coordination/CMakeLists.txt;3;add_test;/home/iannuz/popf-tif/planner/src/popf/testcases/coordination/CMakeLists.txt;0;")
add_test(driverlogshift3-can-be-solved "/home/iannuz/popf-tif/planner/release/popf/popf2" "-I" "-D" "-T" "-v1" "/home/iannuz/popf-tif/planner/src/popf/testcases/coordination/driverlogshift-domain.pddl" "/home/iannuz/popf-tif/planner/src/popf/testcases/coordination/driverlogshift-problem-03.pddl")
set_tests_properties(driverlogshift3-can-be-solved PROPERTIES  _BACKTRACE_TRIPLES "/home/iannuz/popf-tif/planner/src/popf/testcases/coordination/CMakeLists.txt;6;add_test;/home/iannuz/popf-tif/planner/src/popf/testcases/coordination/CMakeLists.txt;0;")
add_test(driverlogshift9-can-be-solved "/home/iannuz/popf-tif/planner/release/popf/popf2" "-I" "-D" "-T" "-v1" "/home/iannuz/popf-tif/planner/src/popf/testcases/coordination/driverlogshift-domain.pddl" "/home/iannuz/popf-tif/planner/src/popf/testcases/coordination/driverlogshift-problem-09.pddl")
set_tests_properties(driverlogshift9-can-be-solved PROPERTIES  _BACKTRACE_TRIPLES "/home/iannuz/popf-tif/planner/src/popf/testcases/coordination/CMakeLists.txt;12;add_test;/home/iannuz/popf-tif/planner/src/popf/testcases/coordination/CMakeLists.txt;0;")
