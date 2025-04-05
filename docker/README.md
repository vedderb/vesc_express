# VESC Express build container
This is a Docker based environment to build your vesc express.

If you don't know what Docker is look here: [Docker in 100 seconds](https://www.youtube.com/watch?v=Gjnup-PuquQ)

The Docker container contains the toolchain and the ESP release 5.2.2 that you need to build vesc express.

If you do not have Docker installed, look here: [Install Docker](#install-docker-engine-in-ubuntu) 

If you are running Windows, look here [Install WSL](#install-wsl)

## Make the Docker image
Using the Dockerfile in this folder you can make an image
```
sudo docker build -t esp-builder:5.2.2 .
```
This will take some time and build a quite large image (~4.1GB)

## Spin up a container
You can then make a container of this image using the command:
```
sudo docker run -it -v ~/vesc:/home/vesc esp-builder:5.2.2 bash
```
You are now inside the container.

_This assumes that you have or want to have VESC related software in a `vesc` folder in your home directory_

(You can use docker-compose or make a Portainer stack in stead of using the `docker run` command above. Google it...)

## Clone source code and build
If you haven't already cloned vesc_express you can do that from inside the container.
```
cd /home/vesc
git clone https://github.com/vedderb/vesc_express.git
```
You will now have a copy of the vesc express source code. 

If you want to do any changes to the code you can do that now. Since the `/home/vesc` folder inside the container is shared with the host operating system, you can do the changes both on the host and in the container. It's probably more convinient to do it on the host.

To build it simply type the following inside the `/home/vesc/vesc_express` folder
```
idf.py build
```
The build process will take some time, and create the bin files that you will use to flash the ESP. 
The bin files are placed inside the `/home/vesc/vesc_express/build` folder.

`vesc_express.bin` is at the root

`bootloader.bin` is inside `bootloader` folder

`partition-table.bin` is inside `partition_table` folder

You can flash your ESP32C3 with these bin files using VESC Tools ESP Programmer.

# Appendix

## Install Docker Engine in Ubuntu
To install Docker Engine in Ubuntu Linux follow these steps:
https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository


## Install WSL
Windows Subsystem for Linux (WSL) is a way to have Linux running inside Windows. Ubuntu is the default Linux distro. To install WSL look here: 
https://learn.microsoft.com/en-us/windows/wsl/install#install-wsl-command

Once installed you can log into Ubuntu using one simple command:
```
wsl 
```
The first time you start you are prompted to create a user

Now [install Docker](#install-docker-engine-in-ubuntu)

The C drive of Windows is available under `/mnt/c`

_Tip:_ If you clone vesc_express here: `C:\Users\username\vesc\vesc_express` the command to start the Docker container would be: 
```
sudo docker run -it -v /mnt/c/Users/username/vesc:/home/vesc esp-builder:5.2.2 bash
```