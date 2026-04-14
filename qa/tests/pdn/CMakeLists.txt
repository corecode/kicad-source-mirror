#
# This program source code file is part of KiCad, a free EDA CAD application.
#
# Copyright The KiCad Developers, see AUTHORS.txt for contributors.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, you may find one here:
# http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
# or you may search the http://www.gnu.org website for the version 2 license,
# or you may write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA


include_directories( BEFORE ${INC_BEFORE} )

include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/pcbnew
    ${CMAKE_SOURCE_DIR}/qa/mocks/include
    ${CMAKE_SOURCE_DIR}/qa/qa_utils
    ${CMAKE_SOURCE_DIR}/qa
    ${INC_AFTER}
    )

set( QA_PDN_SRCS
    test_module.cpp
    test_pdn_end_to_end.cpp
)

find_package( Eigen3 REQUIRED NO_MODULE )

# Find ngspice binary for subprocess invocation
find_program( NGSPICE_EXECUTABLE ngspice )

if( NOT NGSPICE_EXECUTABLE )
    message( WARNING "ngspice binary not found — qa_pdn tests will be skipped" )
endif()

if( WIN32 )
    if( MINGW )
        mingw_resource_compiler( qa_pdn )
    else()
        set( QA_PDN_RESOURCES ${CMAKE_SOURCE_DIR}/resources/msw/qa_spice.rc )
    endif()
endif()

add_executable( qa_pdn
    ${QA_PDN_SRCS}
    ${QA_PDN_RESOURCES}
)

# qa_pdn only links pcbnew so the extractor is available.  ngspice is invoked
# as a subprocess (ngspice -b) so we avoid linking eeschema_kiface_objects
# alongside pcbnew_kiface_objects (which produces duplicate symbols).
target_link_libraries( qa_pdn
PRIVATE
    qa_pcbnew_utils
    pcbnew_kiface_objects
    Eigen3::Eigen
    3d-viewer
    connectivity
    pcbcommon
    pnsrouter
    gal
    common
    scripting
    kimath
    qa_utils
    dxflib_qcad
    tinyspline_lib
    nanosvg
    idf3
    markdown_lib
    ${PCBNEW_IO_LIBRARIES}
    ${wxWidgets_LIBRARIES}
    ${GDI_PLUS_LIBRARIES}
    ${PYTHON_LIBRARIES}
    Boost::headers
    Boost::unit_test_framework
    ${PCBNEW_EXTRA_LIBS}
)

target_compile_definitions( qa_pdn
    PRIVATE
    "NGSPICE_EXECUTABLE_PATH=\"${NGSPICE_EXECUTABLE}\""
)

kicad_add_boost_test( qa_pdn qa_pdn )

setup_qa_env( qa_pdn )
