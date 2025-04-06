FROM ubuntu:latest

# Step 1. Install Prerequisites
RUN : \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        --no-install-recommends \
        git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* \
    && :

# Step 2. Get ESP-IDF v5.2.2
RUN mkdir -p /root/esp/
RUN git clone -b v5.2.2 --recursive https://github.com/espressif/esp-idf.git /root/esp/esp-idf-v5.2.2

# Step 3. Set up the Tools
RUN /root/esp/esp-idf-v5.2.2/install.sh esp32c3

# Step 4. Prepare the build Environment
RUN echo ". /root/esp/esp-idf-v5.2.2/export.sh" > /root/.bashrc