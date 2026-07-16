#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 5 ]]; then
    echo "Usage: $0 <build-dir> <output-dir> <version> <linuxdeploy.AppImage> <runtime>" >&2
    exit 2
fi

build_dir=$(realpath "$1")
output_dir=$(realpath -m "$2")
version=$3
linuxdeploy=$(realpath "$4")
runtime=$(realpath "$5")
repository_root=$(realpath "$(dirname "$0")/../..")
appdir="$output_dir/AppDir"

mkdir -p "$output_dir"
cmake --install "$build_dir" --prefix "$appdir/usr" --component Desktop

export APPIMAGE_EXTRACT_AND_RUN=1
export QMAKE=${QMAKE:-$(command -v qmake6)}
export EXTRA_PLATFORM_PLUGINS=libqoffscreen.so

"$linuxdeploy" \
    --appdir "$appdir" \
    --executable "$appdir/usr/bin/slamforge_desktop" \
    --executable "$appdir/usr/bin/slamforge_cli" \
    --desktop-file "$appdir/usr/share/applications/org.slamforge.SLAMForge.desktop" \
    --icon-file "$appdir/usr/share/icons/hicolor/scalable/apps/org.slamforge.SLAMForge.svg" \
    --plugin qt

mkdir -p "$appdir/usr/share/doc/slamforge"
cp "$repository_root/docs/releases/v${version}.md" \
    "$appdir/usr/share/doc/slamforge/RELEASE_NOTES.md"

pushd "$output_dir" >/dev/null
export LINUXDEPLOY_OUTPUT_VERSION=$version
export LDAI_RUNTIME_FILE=$runtime
export LDAI_NO_APPSTREAM=1
export ARCH=x86_64
"$linuxdeploy" --appdir "$appdir" --output appimage

images=( ./*.AppImage )
if [[ ${#images[@]} -ne 1 ]]; then
    echo "Expected one generated AppImage, found ${#images[@]}" >&2
    exit 1
fi
mv "${images[0]}" "SLAMForge-Desktop-${version}-Linux-x86_64.AppImage"
popd >/dev/null

echo "Created $output_dir/SLAMForge-Desktop-${version}-Linux-x86_64.AppImage"
