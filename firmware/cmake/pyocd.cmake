set(pyocd_dir "${CMAKE_CURRENT_SOURCE_DIR}/../tools/pyocd")
find_program(UV_EXECUTABLE NAMES uv)
if(NOT UV_EXECUTABLE)
    set(UV_EXECUTABLE uv)
endif()
set(pyocd "${UV_EXECUTABLE}" tool run --python 3.12 --managed-python --from pyocd==0.46.0 pyocd)

add_custom_target(pyocd-download
    COMMAND ${pyocd} load --project "${pyocd_dir}" --erase=sector "$<TARGET_FILE:${PROJECT_NAME}>"
    DEPENDS ${PROJECT_NAME}
    USES_TERMINAL VERBATIM
)
add_custom_target(pyocd-gdbserver
    COMMAND ${pyocd} gdbserver --project "${pyocd_dir}" -p 3333 --persist
    USES_TERMINAL VERBATIM
)
