
find_package( OpenThreads )
find_package( osg )
find_package( osgDB )
find_package( osgUtil )
find_package( osgViewer )
find_package(osgGA)       
       
set(OSG_LIBRARIES osg osgViewer OpenThreads osgDB osgGA osgUtil)       
       
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS}
  ${OSG_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

                    
add_library(osgviewer SHARED osgviewer.cpp osgravemain.cpp robot_ui.cpp)
target_link_libraries(osgviewer ${OSG_LIBRARIES} ${OpenRAVE_BOTH_LIBRARIES} ${Boost_SYSTEM_LIBRARY})


add_subdirectory(test)