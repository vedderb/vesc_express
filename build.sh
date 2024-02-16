#!/bin/bash

# Define target configurations
declare -A targets=(
    ["esp32"]="sdkconfig.esp32"
    ["esp32s3"]="sdkconfig.esp32s3"
    ["esp32c3"]="sdkconfig.esp32c3"
)

[ -d "bin" ] || mkdir -p "bin"

# Loop through targets and build
for target in "${!targets[@]}"; do
    config="${targets[$target]}"
    echo "Building for $target using $config..."

    # Clean build directory (optional but recommended to ensure clean builds)
    idf.py fullclean

    # Set the target
    idf.py set-target $target

    # Build the project
	idf.py -D SDKCONFIG=./$config build
    idf.py build
	
	# Rename/move the build output
	cp ./build/vesc_express.bin ./bin/vesc_express.$target.bin
done
