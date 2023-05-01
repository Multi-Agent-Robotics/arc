#!/usr/bin/env bash

yq 'explode(.) | del(.global_parameters)' $1 > $2
