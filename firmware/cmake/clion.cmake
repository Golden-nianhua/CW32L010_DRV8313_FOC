function(cw32_clion_config name target)
    set(CLION_RUN_NAME "${name}")
    set(CLION_BUILD_TARGET "${target}")
    string(REPLACE " " "_" file "${name}")
    configure_file(
        "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/clion-run.xml.in"
        "${CMAKE_CURRENT_SOURCE_DIR}/.idea/runConfigurations/${file}.xml"
        @ONLY NEWLINE_STYLE LF
    )
endfunction()

cw32_clion_config("Build Firmware" "${PROJECT_NAME}")
cw32_clion_config("Download Firmware" pyocd-download)
cw32_clion_config("GDB Server" pyocd-gdbserver)
