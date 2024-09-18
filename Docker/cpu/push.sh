#!/usr/bin/env bash

REPOSITORY="uwwee/ubuntu22.04"
TAG="ros2-cpu"

IMG="${REPOSITORY}:${TAG}"

docker image push "${IMG}"
