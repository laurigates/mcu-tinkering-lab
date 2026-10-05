#!/bin/bash
# MCU Tinkering Lab - ESP32 Project Scaffolding Tool
# Creates a new ESP32 project from template

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PACKAGES_DIR="$REPO_ROOT/packages"

echo -e "${CYAN}MCU Tinkering Lab - ESP32 Project Scaffolding${NC}"
echo "=============================================="
echo

# Get project name
read -p "Enter project name (lowercase, hyphens allowed): " PROJECT_NAME

# Validate project name
if [[ ! $PROJECT_NAME =~ ^[a-z0-9-]+$ ]]; then
    echo -e "${RED}Error: Project name must be lowercase with hyphens only${NC}"
    exit 1
fi

# Pick a domain folder (category)
echo
echo "Select domain folder:"
echo "  1) audio          — audio / synth / toys"
echo "  2) camera-vision  — camera / AI vision projects"
echo "  3) games          — games, scavenger hunts"
echo "  4) input-gaming   — gamepads, controllers, bridges"
echo "  5) networking     — WiFi tests, VPN, network tools"
echo "  6) robocar        — robocar subsystem"
echo "  7) robotics       — motion-control robotics"
echo "  8) sensors        — sensor firmware"
echo "  9) thinkpack      — ThinkPack ESP-NOW mesh boxes"
echo " 10) usb-tools      — USB tools and protocol firmware"
echo " 11) other          — type a custom domain folder name"
read -p "Choice [1-11]: " DOMAIN_CHOICE
case $DOMAIN_CHOICE in
    1) DOMAIN="audio" ;;
    2) DOMAIN="camera-vision" ;;
    3) DOMAIN="games" ;;
    4) DOMAIN="input-gaming" ;;
    5) DOMAIN="networking" ;;
    6) DOMAIN="robocar" ;;
    7) DOMAIN="robotics" ;;
    8) DOMAIN="sensors" ;;
    9) DOMAIN="thinkpack" ;;
    10) DOMAIN="usb-tools" ;;
    11) read -p "Domain folder name: " DOMAIN ;;
    *) echo -e "${RED}Invalid choice${NC}"; exit 1 ;;
esac

PROJECT_DIR="$PACKAGES_DIR/$DOMAIN/$PROJECT_NAME"
mkdir -p "$PACKAGES_DIR/$DOMAIN"

# Check if project already exists
if [ -d "$PROJECT_DIR" ]; then
    echo -e "${RED}Error: Project '$PROJECT_NAME' already exists at $PROJECT_DIR${NC}"
    exit 1
fi

echo
echo "Select project type:"
echo "  1) Basic ESP32 project"
echo "  2) ESP32-CAM project"
echo "  3) ESP32 with OTA support"
echo "  4) Copy from existing project"
read -p "Choice [1-4]: " PROJECT_TYPE

case $PROJECT_TYPE in
    1|2|3)
        # Use cam-webserver as the default template
        TEMPLATE_DIR="$PACKAGES_DIR/camera-vision/cam-webserver"
        ;;
    4)
        # List available projects (<domain>/<name>)
        echo
        echo "Available projects to copy from:"
        find "$PACKAGES_DIR" -mindepth 2 -maxdepth 2 -type d \
            ! -path "*/components/*" ! -name "docs" ! -name "simulation" \
            | sed "s|$PACKAGES_DIR/||"
        echo
        read -p "Enter <domain>/<name> to copy from: " SOURCE_PROJECT
        TEMPLATE_DIR="$PACKAGES_DIR/$SOURCE_PROJECT"

        if [ ! -d "$TEMPLATE_DIR" ]; then
            echo -e "${RED}Error: Source project not found${NC}"
            exit 1
        fi
        ;;
    *)
        echo -e "${RED}Invalid choice${NC}"
        exit 1
        ;;
esac

echo
echo -e "${BLUE}Creating new project: $PROJECT_NAME${NC}"
echo -e "${BLUE}From template: $(basename $TEMPLATE_DIR)${NC}"
echo

# Create project directory
mkdir -p "$PROJECT_DIR"

# Copy template files
echo -e "${CYAN}Copying template files...${NC}"
cp -r "$TEMPLATE_DIR"/* "$PROJECT_DIR/" 2>/dev/null || true

# Remove build artifacts if any
rm -rf "$PROJECT_DIR/build" "$PROJECT_DIR/sdkconfig"

# Update CMakeLists.txt
if [ -f "$PROJECT_DIR/CMakeLists.txt" ]; then
    echo -e "${CYAN}Updating CMakeLists.txt...${NC}"
    TEMPLATE_NAME=$(basename "$TEMPLATE_DIR")
    sed -i "s/$TEMPLATE_NAME/$PROJECT_NAME/g" "$PROJECT_DIR/CMakeLists.txt" 2>/dev/null || \
        sed -i '' "s/$TEMPLATE_NAME/$PROJECT_NAME/g" "$PROJECT_DIR/CMakeLists.txt"
fi

# Update main/CMakeLists.txt
if [ -f "$PROJECT_DIR/main/CMakeLists.txt" ]; then
    echo -e "${CYAN}Updating main/CMakeLists.txt...${NC}"
    TEMPLATE_NAME=$(basename "$TEMPLATE_DIR")
    sed -i "s/$TEMPLATE_NAME/$PROJECT_NAME/g" "$PROJECT_DIR/main/CMakeLists.txt" 2>/dev/null || \
        sed -i '' "s/$TEMPLATE_NAME/$PROJECT_NAME/g" "$PROJECT_DIR/main/CMakeLists.txt"
fi

# Point the copied justfile's project_dir at the new project. Left as is, every
# containerized recipe would still build the template's directory.
if [ -f "$PROJECT_DIR/justfile" ]; then
    sed -i "s|^project_dir := .*|project_dir := \"packages/$DOMAIN/$PROJECT_NAME\"|" "$PROJECT_DIR/justfile" 2>/dev/null || \
        sed -i '' "s|^project_dir := .*|project_dir := \"packages/$DOMAIN/$PROJECT_NAME\"|" "$PROJECT_DIR/justfile"
fi

# Drop `set positional-arguments` from the copied justfile.
# The template is copied wholesale, so this setting propagated into every
# scaffolded project even though no recipe ever read $1/$@ (issue #410).
# A project that genuinely needs positional args should add it back deliberately.
# POSIX awk, not sed: deleting "the blank line after the match" needs either a
# `addr,+1` range or an embedded-newline regex, and both are GNU-only — the BSD
# sed arm this script uses elsewhere would reject them and, under `set -e`,
# abort the scaffold half-written on macOS.
if [ -f "$PROJECT_DIR/justfile" ] && grep -q '^set positional-arguments$' "$PROJECT_DIR/justfile"; then
    echo -e "${CYAN}Cleaning up justfile...${NC}"
    awk '
        /^set positional-arguments$/ { drop = 1; next }
        drop && $0 == ""             { drop = 0; next }
                                     { drop = 0; print }
    ' "$PROJECT_DIR/justfile" > "$PROJECT_DIR/justfile.tmp" &&
        mv "$PROJECT_DIR/justfile.tmp" "$PROJECT_DIR/justfile"
fi

# Create README template
cat > "$PROJECT_DIR/README.md" <<EOF
# $PROJECT_NAME

Description of your ESP32 project.

## Features

- Feature 1
- Feature 2
- Feature 3

## Hardware

- ESP32 board: [Specify your board]
- Additional components: [List any sensors, actuators, etc.]

## Building

Builds run in the ESP-IDF container; no local ESP-IDF install is needed.

\`\`\`bash
just $PROJECT_NAME::build
\`\`\`

## Flashing

\`\`\`bash
PORT=/dev/ttyUSB0 just $PROJECT_NAME::flash
PORT=/dev/ttyUSB0 just $PROJECT_NAME::monitor
\`\`\`

## Configuration

\`\`\`bash
just $PROJECT_NAME::menuconfig
\`\`\`

## License

[Specify license]
EOF

echo
echo -e "${GREEN}✓ Project created successfully!${NC}"
echo
echo "Next steps:"
echo -e "  1. Register the module in the root justfile: ${CYAN}mod $PROJECT_NAME 'packages/$DOMAIN/$PROJECT_NAME'${NC}"
echo -e "  2. Add an entry to ${CYAN}.github/project-matrix.json${NC}: system, project, path, target"
echo -e "     (plus ${CYAN}fetch_bluepad32: true${NC} if it vendors bluepad32)"
echo -e "  3. Check the flash recipe: ${CYAN}python3 tools/check-flash-recipes.py${NC}"
echo -e "  4. Build: ${CYAN}just $PROJECT_NAME::build${NC}"
echo
echo "Optional:"
echo -e "  - Add ${CYAN}flasher.json${NC} to list it in the web flasher (see packages/audio/kids-audio-toy/flasher.json)"
echo
echo "Details: CONTRIBUTING.md § Adding a project, .claude/rules/containerized-builds.md"
