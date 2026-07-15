# ── SLAMForge micro-benchmarks ─────────────────────────────────────────────────

add_executable(bench_orb bench_orb.cpp)
target_link_libraries(bench_orb PRIVATE slamforge_core benchmark::benchmark)
set_project_warnings(bench_orb)

add_executable(bench_pnp bench_pnp.cpp)
target_link_libraries(bench_pnp PRIVATE slamforge_core benchmark::benchmark)
set_project_warnings(bench_pnp)

add_executable(bench_triangulation bench_triangulation.cpp)
target_link_libraries(bench_triangulation PRIVATE slamforge_core benchmark::benchmark)
set_project_warnings(bench_triangulation)
