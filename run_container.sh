#!/usr/bin/env bash
set -euo pipefail

# 1. Define configuration variables
IMAGE_NAME="lidar-framework"
IMAGE_TAG="humble"
CONTAINER_NAME="my-lidar-bench"
TARGET_STAGE="ros-humble"

# Get the name of your current directory (e.g., "lidar_test_bench")
DIR_NAME=$(basename "$(pwd)")
TARGET_MNT="/workspace/${DIR_NAME}"

echo "========================================="
echo "🧹 Cleaning up old containers"
echo "========================================="
# Remove container first so ports/names are completely freed up
docker rm -f "$CONTAINER_NAME" 2>/dev/null || true

echo -e "\n========================================="
echo "🛠️  Building Docker Image (No-Cache): ${IMAGE_NAME}:${IMAGE_TAG}"
echo "========================================="
# Dockerfile now lives in .devcontainer/ (shared with the VS Code dev container).
# Build context stays "." so `COPY requirements.txt*` still resolves.
docker build --target "$TARGET_STAGE" -t "${IMAGE_NAME}:${IMAGE_TAG}" -f .devcontainer/Dockerfile .

echo -e "\n========================================="
echo "🚀 Launching Container: ${CONTAINER_NAME}"
echo "   Mounting entire directory to: ${TARGET_MNT}"
echo "========================================="
docker run -it --rm \
  --net=host \
  --ipc=host \
  -v "$(pwd)":"${TARGET_MNT}" \
  -w "${TARGET_MNT}" \
  --name "$CONTAINER_NAME" \
  "${IMAGE_NAME}:${IMAGE_TAG}"
