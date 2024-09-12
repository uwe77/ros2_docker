#!/usr/bin/env bash

REPOSITORY="uwwee/ubuntu22.04"
TAG="ros2-humble"

IMG="${REPOSITORY}:${TAG}"

docker image push "${IMG}"
