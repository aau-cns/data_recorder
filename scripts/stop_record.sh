#!/bin/bash

NODES=$(rosnode list | grep "/record_")

if [[ ! -z ${NODES} ]]; then
  echo ${NODES} | xargs rosnode kill
else
  echo "[RECORD_STOP] already all recordings stop -- skipping redundant killing"
fi
