# CMake generated Testfile for 
# Source directory: /home/syz/move_base_qianjin/src/navigation-noetic-devel/base_local_planner
# Build directory: /home/syz/move_base_qianjin/src/navigation-noetic-devel/base_local_planner/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_base_local_planner_gtest_base_local_planner_utest "/home/syz/move_base_qianjin/src/navigation-noetic-devel/base_local_planner/build/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/syz/move_base_qianjin/src/navigation-noetic-devel/base_local_planner/build/test_results/base_local_planner/gtest-base_local_planner_utest.xml" "--return-code" "/home/syz/move_base_qianjin/src/navigation-noetic-devel/base_local_planner/build/devel/lib/base_local_planner/base_local_planner_utest --gtest_output=xml:/home/syz/move_base_qianjin/src/navigation-noetic-devel/base_local_planner/build/test_results/base_local_planner/gtest-base_local_planner_utest.xml")
add_test(_ctest_base_local_planner_gtest_line_iterator "/home/syz/move_base_qianjin/src/navigation-noetic-devel/base_local_planner/build/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/syz/move_base_qianjin/src/navigation-noetic-devel/base_local_planner/build/test_results/base_local_planner/gtest-line_iterator.xml" "--return-code" "/home/syz/move_base_qianjin/src/navigation-noetic-devel/base_local_planner/build/devel/lib/base_local_planner/line_iterator --gtest_output=xml:/home/syz/move_base_qianjin/src/navigation-noetic-devel/base_local_planner/build/test_results/base_local_planner/gtest-line_iterator.xml")
subdirs("gtest")
