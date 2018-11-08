add_library(visualization SHARED visualizer.cpp)
target_link_libraries(visualization
                      identifiable
                      ${QGLVIEWER_LIBRARY}
                      ${MY_QT_LIBRARIES}
                      ${OPENGL_gl_LIBRARY}
                      ${OPENGL_glu_LIBRARY})