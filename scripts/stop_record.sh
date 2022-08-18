#!/bin/bash
# Copyright (C) 2022 Martin Scheiber, Control of Networked Systems, University of Klagenfurt, Austria.
#
# All rights reserved.
#
# This software is licensed under the terms of the BSD-2-Clause-License with
# no commercial use allowed, the full terms of which are made available
# in the LICENSE file. No license in patents is granted.
#
# You can contact the author at <martin.scheiber@ieee.org>

NODES=$(rosnode list | grep "/record_")

if [[ ! -z ${NODES} ]]; then
  echo ${NODES} | xargs rosnode kill
else
  echo "[RECORD_STOP] already all recordings stop -- skipping redundant killing"
fi
