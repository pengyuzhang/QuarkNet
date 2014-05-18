#!/bin/sh

sudo make clean
sudo ./bootstrap
sudo ./configure
sudo make
sudo make install
