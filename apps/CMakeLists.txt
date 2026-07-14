# ── LiteVO CLI application ──────────────────────────────────────────────────

add_executable(litevo_cli
    litevo_cli.cpp
)

target_link_libraries(litevo_cli PRIVATE
    litevo_core
    CLI11::CLI11
)

# Workaround: OpenCV's highgui/imgcodecs pull in GDAL which needs curl
find_package(CURL QUIET)
if(CURL_FOUND)
    target_link_libraries(litevo_cli PRIVATE CURL::libcurl)
endif()

set_project_warnings(litevo_cli)

# Install the CLI application
install(TARGETS litevo_cli
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
