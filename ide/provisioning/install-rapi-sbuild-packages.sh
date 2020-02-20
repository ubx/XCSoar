#!/bin/sh

set -e

export DEBIAN_FRONTEND=noninteractive
APTOPTS="--assume-yes --no-install-recommends"

echo "To install a (chrootable) Raspbian sbuild sysroot,"
echo "install qemu-user-static sbuild on your host..."
apt-get install $APTOPTS qemu-user-static sbuild
sbuild-adduser $(whoami)
sbuild-createchroot --arch=armhf --include=gnupg,libinput-dev,libasound2-dev,libcurl4-openssl-dev,liblua5.2-dev, \
    zlib1g-dev,libfreetype6-dev,libtiff5-dev,libgeotiff-dev,libsocketcan-dev \
    --debootstrap=qemu-debootstrap --chroot-suffix=-raspbian --keyring=stretch /chroots/stretch-armhf-raspbian http://mirrordirector.raspbian.org/raspbian/

echo "Correct the sysroot's sources.list..."
sed -i "s,http://deb.debian.org/debian,http://mirrordirector.raspbian.org/raspbian/,g" /chroots/stretch-armhf-raspbian/etc/apt/sources.list
echo deb http://archive.raspberrypi.org/debian/ stretch main | tee -a /chroots/stretch-armhf-raspbian/etc/apt/sources.list

echo "Install Raspbian's public keys..."
wget -O - https://archive.raspbian.org/raspbian.public.key | schroot -c source:stretch-armhf-raspbian -u root -d /tmp -- apt-key add -
wget -O - https://archive.raspberrypi.org/debian/raspberrypi.gpg.key | schroot -c source:stretch-armhf-raspbian -u root -d /tmp -- apt-key add -

echo "Install the libraspberrypi-dev package in your sysroot..."
sbuild-update -udcar stretch-armhf-raspbian
sbuild-apt stretch-armhf-raspbian apt-get install libraspberrypi-dev

echo "Unfortunately, there are some absolute library symlinks in the sysroot. Correct it..."
cd /chroots/stretch-armhf-raspbian/usr/lib/arm-linux-gnueabihf
for i in *.so*; do if [ -h $i ] && [ ! -e $i ]; then j=`readlink $i`; if [[ $j == /* ]]; then ln -s -f ../../..$j $i; fi; fi; done