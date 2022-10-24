# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/eigen3;/opt/ros/noetic/include".split(';') if "${prefix}/include;/usr/include/eigen3;/opt/ros/noetic/include" != "" else []
PROJECT_CATKIN_DEPENDS = "cubic_polynomial_planner;roscpp".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lcubic_polynomial_planner;-lpinocchio;-lEigen3".split(';') if "-lcubic_polynomial_planner;-lpinocchio;-lEigen3" != "" else []
PROJECT_NAME = "inverse_kinematic_controller"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
