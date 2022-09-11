#!/bin/bash

# Replaces unnecessary -I with -isystem

clean_build_db() {
    local original=$(cat build/compile_commands.json)
    sed -i 's|-I/opt/|-isystem /opt/|g' build/compile_commands.json
    sed -i 's|-I/usr/|-isystem /usr/|g' build/compile_commands.json
    sed -i 's|-I\(.*/cmr_msgs/\)|-isystem \1|g' build/compile_commands.json
    local edited=$(cat build/compile_commands.json)
    if [ "$original" = "$edited" ]; then
        echo "1"
    else
        echo "0"
    fi
}

code=$(clean_build_db)
while [ "$code" != "1" ]; do
    code=$(clean_build_db)
done