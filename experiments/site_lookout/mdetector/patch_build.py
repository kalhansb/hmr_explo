"""Build-file changes only: TBB from the system (not a local oneTBB build),
no livox_ros_driver (only needed for Livox message input), and the offline
driver as an extra executable."""
import re, sys
d = sys.argv[1]
c = open(f'{d}/CMakeLists.txt').read()
c = re.sub(r'set\(TBB_DIR.*?\n', '', c)
c = re.sub(r'include\("/home/huajie.*?\n', '', c)
c = re.sub(r'tbb_build\(.*?\n', '', c)
c = c.replace('  livox_ros_driver\n', '')
c = c.replace('find_package(TBB REQUIRED)', 'find_library(TBB_LIB tbb REQUIRED)')
c = c.replace('TBB::tbb', '${TBB_LIB}')
c += '''
add_executable(md_offline src/md_offline.cpp src/DynObjFilter.cpp src/DynObjCluster.cpp)
target_link_libraries(md_offline ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${OpenCV_LIBS} ${TBB_LIB})
target_include_directories(md_offline PRIVATE ${PYTHON_INCLUDE_DIRS})
'''
open(f'{d}/CMakeLists.txt', 'w').write(c)
p = open(f'{d}/package.xml').read()
p = re.sub(r'\s*<(build|run)_depend>livox_ros_driver</\1_depend>', '', p)
open(f'{d}/package.xml', 'w').write(p)
