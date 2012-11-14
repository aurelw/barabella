include_directories (../core/)

### PCL 3d view ###
set (gui_source_files view3d.cpp)


### Qt GUI ###
FIND_PACKAGE(Qt4 REQUIRED)
set (QT_USE_QTOPENGL TRUE)
include (${QT_USE_FILE})
add_definitions (${QT_DEFINITIONS})
include_directories (${CMAKE_CURRENT_BINARY_DIR})
# also include cwd, since complilation of created files by moc
# happens in a different directory.
include_directories (.)

set (qtgui_SOURCES pclcloudwidget.cpp barbarellauitest.cpp maincloudwidget.cpp)
set (qtgui_HEADERS pclcloudwidget.h barbarellauitest.h maincloudwidget.h)
set (qtgui_FORMS barbarellauitest.ui)
set (qtgui_RESOURCES barbarellauitest.qrc)

QT4_ADD_RESOURCES(qtgui_RESOURCES_RCC ${qtgui_RESOURCES})
QT4_WRAP_CPP(qtgui_HEADERS_MOC ${qtgui_HEADERS})
QT4_WRAP_UI(qtgui_FORMS_HEADERS ${qtgui_FORMS})

add_library (gui ${gui_source_files} ${qtgui_FORMS_HEADERS} ${qtgui_HEADERS_MOC} ${qtgui_SOURCES} ${qtgui_RESOURCES_RCC})

target_link_libraries (gui core ${QT_LIBRARIES})

### VTK ###
FIND_PACKAGE(VTK)

add_executable (qtgui main.cpp)
target_link_libraries (qtgui QVTK gui)

