# Contributing to SLAMForge

Thanks for your interest in contributing! SLAMForge is a monocular visual SLAM and dense
reconstruction system, and we welcome improvements.

## Development Setup

```bash
git clone https://github.com/JackXing875/SLAMForge.git
cd SLAMForge

# Build with tests
cmake -B build -DSLAMFORGE_BUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)

# Run tests
cd build && ctest --output-on-failure
```

## Code Style

- **C++20** standard
- **clang-format** with `.clang-format` config at root
- **clang-tidy** for static analysis
- All public APIs must have `/// @brief` Doxygen comments
- Use `PascalCase` for classes, `snake_case` for functions/variables
- Namespaces: `slamforge::{subsystem}`

## Commit Guidelines

Use conventional commits:
- `feat(tracking): add motion-only BA with Ceres`
- `fix(mapping): handle empty covisibility graph`
- `docs: update architecture diagram`
- `test: add map point culling tests`

## Pull Request Process

1. Create a feature branch: `feature/description`
2. Write or update tests for your changes
3. Ensure `clang-format` and `clang-tidy` pass
4. Run `ctest` — all tests must pass
5. Open a PR against `main` with a description of changes
6. CI will automatically build, test, and lint

## Testing

- Unit tests in `tests/unit/` use GoogleTest
- Each test file compiles as a separate executable
- Run with: `cd build && ctest --output-on-failure`

## Documentation

- API docs: `/// @brief` comments on all public classes/methods
- Build docs: `doxygen Doxyfile` → opens in `docs/html/`
- Architecture docs in `docs/architecture.md`

## Project Structure

```
include/slamforge/   — Public headers (one per module)
src/               — Implementation files
apps/              — CLI and ROS2 applications
tests/unit/        — Unit tests
tests/bench/       — Micro-benchmarks
tools/             — Python evaluation utilities
pybind/            — Python bindings
config/            — YAML configuration files
docker/            — Docker configuration
docs/              — Documentation
```

## Questions?

Open an issue or discussion on GitHub.
