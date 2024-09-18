#!/usr/bin/env bash

REPOSITORY="uwwee/ubuntu22.04"
TAG="ros2-gpu"

IMG="${REPOSITORY}:${TAG}"

docker image push "${IMG}"
