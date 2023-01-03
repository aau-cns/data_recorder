#!/bin/bash
# Copyright (C) 2023 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
#
# All rights reserved.
#
# This software is licensed under the terms of the BSD-2-Clause-License with
# no commercial use allowed, the full terms of which are made available
# in the LICENSE file. No license in patents is granted.
#
# You can contact the author at <martin.scheiber@ieee.org>

echo "[BASH] Called 'store.sh' with arguments"
echo "       1: ${1}"
echo "       2: ${3}"
echo "       3: ${2}"

# perform while loop with sleeps of 1s (max of 10s)
COUNTER=0
while [ $COUNTER -lt 10 ]; do
    echo "[BASH] store: stored another 1s"
    let COUNTER+=1
    sleep 1
done

exit 0
