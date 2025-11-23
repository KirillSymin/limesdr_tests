FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC

# Basic tools needed to add the MyriadRF PPA key
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        gnupg \
        wget && \
    rm -rf /var/lib/apt/lists/*

# Add MyriadRF "drivers" PPA (contains LimeSuite 23.11.0-1 for Ubuntu 22.04)
# https://launchpad.net/~myriadrf/+archive/ubuntu/drivers 
RUN echo "deb [signed-by=/usr/share/keyrings/myriadrf-archive-keyring.gpg] http://ppa.launchpad.net/myriadrf/drivers/ubuntu jammy main" \
      > /etc/apt/sources.list.d/myriadrf-drivers.list && \
    export GNUPGHOME=/tmp/gnupg && mkdir -p "$GNUPGHOME" && chmod 700 "$GNUPGHOME" && \
    gpg --batch --keyserver keyserver.ubuntu.com \
        --recv-keys 11FC2E68126782B43762694F22C627172ECB91FE && \
    gpg --batch --export 11FC2E68126782B43762694F22C627172ECB91FE \
        > /usr/share/keyrings/myriadrf-archive-keyring.gpg && \
    rm -rf "$GNUPGHOME"

# Install:
#   - C toolchain only (gcc, no g++)
#   - LimeSuite + dev headers + udev rules + images
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        # C toolchain (no C++):
        gcc-11=11.4.0-1ubuntu1~22.04.2 \
        make=4.3-4.1build1 \
        libc6-dev \
        pkg-config \
        libusb-1.0-0-dev \
        # LimeSuite classic from MyriadRF PPA:
        # matches LimeUtil: Library version v23.11.0-1 on your host
        limesuite=23.11.0-1 \
        liblimesuite-dev=23.11.0-1 \
        limesuite-udev=23.11.0-1 \
        limesuite-images=23.11.0-1 \
    && rm -rf /var/lib/apt/lists/*

# Optional: make "gcc" point to gcc-11 for convenience
RUN ln -s /usr/bin/gcc-11 /usr/bin/gcc

# Where you’ll work (mount your code here)
WORKDIR /workspace

# Interactive mode: nothing runs until you attach
CMD ["/bin/bash"]
