# Copyright (c) 2024-2025 Voyant Photonics, Inc.
#
# This example code is licensed under the MIT License.
# See the LICENSE file in the repository root for full license text.

# cmake/Warnings.cmake
function(set_project_warnings project_name)
    set(MSVC_WARNINGS
        /W4 # Baseline reasonable warnings
        /WX # Treat warnings as errors
    )

    set(CLANG_WARNINGS
        -Wall
        -Wextra
        -Werror
        -Wconversion
        -Wno-sign-conversion
        -fcolor-diagnostics
    )

    set(GCC_WARNINGS
        ${CLANG_WARNINGS}
        -fdiagnostics-color=always
        -Wpedantic
        -Wcast-align
        -Wcast-qual
        -Wdisabled-optimization
        -Woverloaded-virtual
    )

    if(MSVC)
        target_compile_options(${project_name} INTERFACE ${MSVC_WARNINGS})
        target_compile_definitions(${project_name} INTERFACE _USE_MATH_DEFINES)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
        target_compile_options(${project_name} INTERFACE ${CLANG_WARNINGS})
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options(${project_name} INTERFACE ${GCC_WARNINGS})
    endif()
endfunction()


# cmake/CompilerSettings.cmake
function(setup_modern_cpp target)
    target_compile_features(${target} PUBLIC cxx_std_17)
    if(MSCV)
        target_compile_definitions(${target} PRIVATE _USE_MATH_DEFINES)
        target_compile_options(${target} PRIVATE /W4 /WX)
    else()
        target_compile_options(${target} PRIVATE
            -Wall -Wextra -Werror -pedantic
            $<$<CXX_COMPILER_ID:GNU>:-Wcast-align -Wcast-qual -Wconversion -Wdisabled-optimization -Woverloaded-virtual>
            $<$<CXX_COMPILER_ID:Clang>:-Wconversion -Wno-sign-conversion>
        )

        # Colored output
        if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            target_compile_options(${target} PRIVATE -fdiagnostics-color=always)
        elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
            target_compile_options(${target} PRIVATE -fcolor-diagnostics)
        endif()
    endif()

    target_include_directories(${target}
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
        PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )
endfunction()
