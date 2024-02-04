project(mars_model_tests)

# ###############################
# GTest
# ###############################
include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.12.0
)
FetchContent_MakeAvailable(googletest)
add_library(GTest::GTest INTERFACE IMPORTED)
target_link_libraries(GTest::GTest INTERFACE gtest_main)

# ###############################
# Unit Tests
# ###############################
macro(package_add_test TESTNAME)
    # create an executable in which the tests will be stored
    add_executable(${TESTNAME} ${ARGN})

    # link the Google test infrastructure, mocking library, and a default main function to
    # the test executable.  Remove g_test_main if writing your own main function.
    target_link_libraries(${TESTNAME}
        GTest::GTest
        Eigen3::Eigen 
        pinocchio::pinocchio
        ${CMAKE_DL_LIBS}
    )

    set_target_properties(${TESTNAME} PROPERTIES FOLDER tests)
    add_test(${TESTNAME} ${TESTNAME})
endmacro()

# add tests here:
package_add_test(test_inverse_kinematics test_inverse_kinematics.cpp)
package_add_test(test_ik_limits test_ik_limits.cpp)