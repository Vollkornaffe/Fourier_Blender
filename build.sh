#!/bin/bash

swig -c++ -python example.i
g++ -fpic -shared *.cxx -I/usr/local/include/eigen3/ -I/usr/include/python3.6 -o _example.so
