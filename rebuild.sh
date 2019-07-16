#!/bin/bash

rm -r *_isolated
catkin_make_isolated --install --use-ninja
