#!/bin/bash

exit 0

./waf configure

./waf build size upload --app=10
