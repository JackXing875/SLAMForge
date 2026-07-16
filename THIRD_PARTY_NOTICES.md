# Third-Party Notices

SLAMForge Desktop is GPL-3.0-only software and dynamically or statically uses third-party open-source
components. The release packages include the applicable license texts where provided by their
package managers. The principal components are listed below; the linked upstream projects remain
the authoritative source for their complete notices and dependency trees.

| Component | Purpose | License |
|---|---|---|
| Qt 6 Core/Gui/Widgets | Desktop interface | LGPL-3.0-only / GPL options |
| OpenCV | Video decoding and computer vision | Apache-2.0 |
| Eigen | Linear algebra | MPL-2.0 |
| Ceres Solver | Bundle adjustment | BSD-3-Clause |
| spdlog | Logging | MIT |
| yaml-cpp | Configuration | MIT |
| CLI11 | Command-line interface | BSD-3-Clause |

The optional Sophus, g2o, and FBOW backends are source-tree capabilities but are not enabled in the
`3.1.0-beta.2` desktop binaries. GoogleTest and other test/build tools are not shipped as runtime
components.

Qt is dynamically linked in the desktop packages. Users may replace the compatible Qt shared
libraries in accordance with LGPL-3.0. Complete corresponding SLAMForge source is available from the
GitHub tag associated with each binary release:

<https://github.com/JackXing875/SLAMForge/releases>
