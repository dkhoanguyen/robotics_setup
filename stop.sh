#!/bin/bash

# Add the container names you want to keep to this array
container_names_to_keep=("container_name1" "container_name2" "container_name3")

# Get all running container IDs
all_container_ids=$(docker ps -q)

# Loop through the container IDs and check if their names are in the "to keep" list
container_ids_to_stop=()
for container_id in $all_container_ids; do
    container_name=$(docker inspect -f '{{.Name}}' "$container_id" | sed 's,^/,,')
    if [[ ! " ${container_names_to_keep[@]} " =~ " ${container_name} " ]]; then
        container_ids_to_stop+=("$container_id")
    fi
done

# Stop the containers (if any)
if [[ ${#container_ids_to_stop[@]} -gt 0 ]]; then
    docker stop "${container_ids_to_stop[@]}"
    echo "Stopped all containers except: ${container_names_to_keep[*]}."
else
    echo "No containers to stop except: ${container_names_to_keep[*]}."
fi
