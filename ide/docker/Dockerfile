FROM debian:buster
RUN echo 'deb http://deb.debian.org/debian stretch main' > /etc/apt/sources.list.d/unstable.list
RUN echo 'APT::Default-Release "stable";' > /etc/apt/apt.conf.d/99defaultrelease

ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies
COPY ./provisioning/install-debian-packages.sh /root/install-debian-packages.sh
RUN /root/install-debian-packages.sh && apt-get clean
COPY ./provisioning/install-android-tools.sh /root/install-android-tools.sh
RUN chmod 755 /root/install-android-tools.sh /root/install-debian-packages.sh
RUN /root/install-android-tools.sh && apt-get clean
RUN apt-get -y install python-minimal python3 vim ccache python3-pkgconfig && apt-get clean

# Installing dependencies for Toradex Colibri T30 Linux targets
RUN wget https://developer.toradex.com/files/toradex-dev/uploads/media/Colibri/Linux/SDKs/2.8/colibri-t30/angstrom-lxde-image/angstrom-glibc-x86_64-armv7at2hf-neon-v2017.12-toolchain.sh
RUN chmod +x angstrom-glibc-x86_64-armv7at2hf-neon-v2017.12-toolchain.sh
RUN ./angstrom-glibc-x86_64-armv7at2hf-neon-v2017.12-toolchain.sh -y
RUN rm /angstrom-glibc-x86_64-armv7at2hf-neon-v2017.12-toolchain.sh

WORKDIR /opt/xcsoar

COPY ./docker/bin/* /usr/local/bin/
RUN cd /usr/local/bin && chmod 755 ./*
