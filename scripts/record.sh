#!/bin/bash

echo "[BASH] Called 'record.sh'"

# perform while loop with sleeps of 2s (max of 2000s
COUNTER=0
while [ $COUNTER -lt 1000 ]; do
    echo "[BASH] rec: recorded another 2s"
    let COUNTER+=1
    sleep 2
done

exit 0
