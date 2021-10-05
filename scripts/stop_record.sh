#!/bin/bash

rosnode list | grep "/record_" | xargs rosnode kill
