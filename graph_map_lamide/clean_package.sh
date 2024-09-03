#!/bin/bash

echo "Script for cleaning binaries from repo.";
echo "Author: Kevin Li Sun, Henry Cheng Zhao, Jan-2016"
echo

echo "Removing backup files."
find ./ -name '*~' | xargs rm
find ./ -name '*.pyc' | xargs rm
