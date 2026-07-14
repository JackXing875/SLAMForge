# -----------------------------------------------------------------------------
# Compiler warning configuration for LiteVO
# -----------------------------------------------------------------------------

function(set_project_warnings target_name)
    set(MSVC_WARNINGS
        /W4
        /permissive-
        /w14242  # 'identifier': conversion from 'type1' to 'type2', possible loss
        /w14254  # 'operator': conversion from 'type1' to 'type2', possible loss
        /w14263  # 'function': member function does not override any base class
        /w14265  # 'classname': class has virtual functions, but destructor is not
        /w14287  # 'operator': unsigned/negative constant mismatch
        /w14296  # 'operator': expression is always 'boolean_value'
        /w14311  # 'variable': pointer truncation from 'type1' to 'type2'
        /w14545  # expression before comma evaluates to a function which is missing
        /w14546  # function call before comma missing argument list
        /w14547  # 'operator': operator before comma has no effect
        /w14549  # 'operator': operator before comma has no effect
        /w14555  # expression has no effect; expected expression with side-effect
        /w14826  # Conversion from 'type1' to 'type2' is sign-extended
        /w14905  # wide string literal cast to 'LPSTR'
        /w14906  # string literal cast to 'LPWSTR'
        /w14928  # illegal copy-initialization; more than one user-defined conversion
    )

    set(CLANG_WARNINGS
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Wcast-align
        -Wunused
        -Woverloaded-virtual
        -Wconversion
        -Wsign-conversion
        -Wnull-dereference
        -Wdouble-promotion
        -Wformat=2
        -Wmisleading-indentation
        -Wimplicit-fallthrough
        -Wno-unknown-warning-option
    )

    set(GCC_WARNINGS
        ${CLANG_WARNINGS}
        -Wduplicated-cond
        -Wduplicated-branches
        -Wlogical-op
        -Wuseless-cast
        -Wno-unknown-pragmas
    )

    if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
        set(PROJECT_WARNINGS ${MSVC_WARNINGS})
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        set(PROJECT_WARNINGS ${CLANG_WARNINGS})
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
        set(PROJECT_WARNINGS ${GCC_WARNINGS})
    endif()

    target_compile_options(${target_name} INTERFACE ${PROJECT_WARNINGS})
endfunction()
