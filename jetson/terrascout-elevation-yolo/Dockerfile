FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Install Python virtual environment support
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-venv \
        python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Create a Python environment that can access ROS 2 packages
RUN python3 -m venv --system-site-packages /opt/venv

ENV PATH="/opt/venv/bin:${PATH}"

# Install CuPy and compatible NumPy
RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir \
        "numpy<2.0" \
        cupy-cuda12x

WORKDIR /workspace

CMD ["bash"]
