# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Sean Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)
file(GLOB conf    ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
                 
### create a single target that installs all applications at once
yarp_install(FILES ${conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/stereoVision)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
