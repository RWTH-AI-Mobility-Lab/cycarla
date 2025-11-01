#!/bin/bash

echo "🚀 Starting build process for cycarla-agent..."

# Change to the script's directory
cd "$(dirname "$0")"

# Use uv for building
BUILD_CMD="uv run python"

# Create build directory if it doesn't exist
mkdir -p build

# Detect number of CPU cores
NUM_CORES=$(nproc)

# Build the package
echo "🔨 Building package using $NUM_CORES cores..."
$BUILD_CMD -m nuitka \
    --standalone \
    --onefile \
    --follow-imports \
    --output-dir=build \
    --output-filename=cycarla_agent.bin \
    --jobs=$NUM_CORES \
    --lto=yes \
    src/cycarla_agent

if [ $? -eq 0 ]; then
    echo "✅ Build completed successfully!"
    echo "📁 Output binary: $(pwd)/build/cycarla_agent.bin"
else
    echo "❌ Build failed!"
    exit 1
fi