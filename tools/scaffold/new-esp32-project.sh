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
ESP32_PROJECTS_DIR="$REPO_ROOT/packages/esp32-projects"

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

PROJECT_DIR="$ESP32_PROJECTS_DIR/$PROJECT_NAME"

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
        # Use esp32-cam-webserver as template for now
        TEMPLATE_DIR="$ESP32_PROJECTS_DIR/esp32-cam-webserver"
        ;;
    4)
        # List available projects
        echo
        echo "Available projects to copy from:"
        ls -1 "$ESP32_PROJECTS_DIR" | grep -v "robocar-docs" | grep -v "robocar-simulation"
        echo
        read -p "Enter project name to copy from: " SOURCE_PROJECT
        TEMPLATE_DIR="$ESP32_PROJECTS_DIR/$SOURCE_PROJECT"

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

\`\`\`bash
# Build project
cd packages/esp32-projects/$PROJECT_NAME
idf.py build

# Or use root Makefile (add targets first)
# make $PROJECT_NAME-build
\`\`\`

## Flashing

\`\`\`bash
idf.py flash -p /dev/ttyUSB0
idf.py monitor -p /dev/ttyUSB0
\`\`\`

## Configuration

\`\`\`bash
idf.py menuconfig
\`\`\`

## License

[Specify license]
EOF

echo
echo -e "${GREEN}✓ Project created successfully!${NC}"
echo
echo "Next steps:"
echo -e "  1. ${CYAN}cd $PROJECT_DIR${NC}"
echo -e "  2. ${CYAN}Edit main/main.c to implement your application${NC}"
echo -e "  3. ${CYAN}Update README.md with project details${NC}"
echo -e "  4. ${CYAN}idf.py build${NC} to build the project"
echo
echo "Optional:"
echo -e "  - Add to root Makefile for easy building: ${CYAN}make $PROJECT_NAME-build${NC}"
echo -e "  - Add to CI pipeline: ${CYAN}.github/workflows/esp32-build.yml${NC}"
echo
echo -e "${GREEN}Happy coding! 🚀${NC}"
