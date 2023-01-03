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

echo "[BASH] Called 'record.sh' with arguments"
echo "       1: ${1}"
echo "       2: ${3}"
echo "       3: ${2}"


# perform while loop with sleeps of 2s (max of 2000s)
COUNTER=0
while [ $COUNTER -lt 1000 ]; do
    echo "[BASH] rec: recorded another 2s"
    let COUNTER+=1
    sleep 2
done

exit 0
