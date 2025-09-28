#!/bin/bash

# ESP32-CAM LLM Telegram Build Script

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}ESP32-CAM LLM Telegram Builder${NC}"
echo "================================="

# Check if IDF_PATH is set
if [ -z "$IDF_PATH" ]; then
    echo -e "${RED}Error: IDF_PATH not set. Please source ESP-IDF environment first.${NC}"
    echo "Run: . \$IDF_PATH/export.sh"
    exit 1
fi

# Set default port
PORT=${1:-/dev/ttyUSB0}

# Menu
echo "Select action:"
echo "1) Build only"
echo "2) Build and flash"
echo "3) Flash only"
echo "4) Monitor"
echo "5) Build, flash and monitor"
echo "6) Clean build"
echo "7) Menuconfig"
echo "8) Size analysis"

read -p "Choice [1-8]: " choice

case $choice in
    1)
        echo -e "${YELLOW}Building project...${NC}"
        idf.py build
        ;;
    2)
        echo -e "${YELLOW}Building and flashing to $PORT...${NC}"
        idf.py build flash -p $PORT
        ;;
    3)
        echo -e "${YELLOW}Flashing to $PORT...${NC}"
        idf.py flash -p $PORT
        ;;
    4)
        echo -e "${YELLOW}Starting monitor on $PORT...${NC}"
        idf.py monitor -p $PORT
        ;;
    5)
        echo -e "${YELLOW}Building, flashing and monitoring on $PORT...${NC}"
        idf.py build flash monitor -p $PORT
        ;;
    6)
        echo -e "${YELLOW}Cleaning build directory...${NC}"
        idf.py fullclean
        echo -e "${GREEN}Clean complete!${NC}"
        ;;
    7)
        echo -e "${YELLOW}Opening menuconfig...${NC}"
        idf.py menuconfig
        ;;
    8)
        echo -e "${YELLOW}Analyzing binary size...${NC}"
        idf.py size
        idf.py size-components
        ;;
    *)
        echo -e "${RED}Invalid choice!${NC}"
        exit 1
        ;;
esac

echo -e "${GREEN}Done!${NC}"