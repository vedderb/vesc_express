# VESC Express build container
This is a docker based environment to build your vesc express.

The docker container contains the toolchain and the ESP release 5.2.2 that you need to build vesc express.

## Make the docker image
Using the Dockerfile in this folder you can make an image
```
sudo docker build -t esp-builder:5.2.2 .
```
This will take some time and build a quite large image (~4.1GB)

## Spin up a container
You can then make a container of this image using either the command:
```
sudo docker run -it -v ~/vesc:/home/vesc esp-builder:5.2.2 bash
```
You are now inside the container

(Alternatively, use docker-compose or make a Portainer stack)

## Clone source code and build
While inside the container, clone the version of the vesc express source you want.

Forexample the main branch from vedderb (the official version):
```
cd /home/vesc
git clone https://github.com/vedderb/vesc_express.git
```
You will now have a copy of the vesc express source code. 

If you want to do any changes to the code you can do that now. Since the `/home/vesc` folder inside the container is shared with the host operating system, so you can do the changes both on the host and in the container.

To build it simply type the following inside the `vesc_express` folder
```
idf.py build
```
The build process will take some time, and create the bin files that you will use to flash the ESP. Look inside the `build` folder for `vesc_express.bin` 

`bootloader.bin` is inside `build/bootloader`

`partition-table.bin` is inside `build/partition_table`
