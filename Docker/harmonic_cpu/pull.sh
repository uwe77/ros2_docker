#!/usr/bin/env bash

REPOSITORY="uwwee/ubuntu22.04"
TAG="ros2-humble-harmonic-cpu"

IMG="${REPOSITORY}:${TAG}"

docker pull "${IMG}"