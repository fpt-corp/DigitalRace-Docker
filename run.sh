#!/bin/bash
sudo docker build --tag=<tên image> .

sudo docker run --rm -it --gpus=all --network=host --memory=6000M -d -v ~/<đường dẫn nối volume>:/root/catkin_ws/src/ --name <tên đội> <tên image> bash
