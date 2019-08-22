TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += /opt/ros/melodic/include/ \
               /home/mat/work/ros/devel/include \
               /home/mat/work/ros/src/multi_agent_planner/src

SOURCES += \
        ../src/agent_node.cpp \
        ../src/multi_agent_planner_node.cpp \
        ../src/planner.cpp \
        ../src/roadmap.cpp \
        ../src/sampleagent.cpp \
        ../src/testcase.cpp

HEADERS += \
    ../src/planner.h \
    ../src/roadmap.h \
    ../src/sampleagent.h
