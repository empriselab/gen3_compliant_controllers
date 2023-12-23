#!/bin/bash

set -ex

cd /workspace/build/gen3_compliant_controllers/
#Check whether files match format
make check-format