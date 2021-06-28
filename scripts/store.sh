#!/bin/bash

echo "[BASH] Called 'store.sh' with argument ${1}"

# perform while loop with sleeps of 1s (max of 10s)
COUNTER=0
while [ $COUNTER -lt 10 ]; do
    echo "[BASH] store: stored another 1s"
    let COUNTER+=1
    sleep 1
done

exit 0
