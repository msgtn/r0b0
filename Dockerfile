FROM ros:jazzy-ros-base
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/


COPY ./r0b0_interfaces /r0b0/r0b0_interfaces
WORKDIR /r0b0
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

COPY . /r0b0
RUN uv sync
RUN uv pip install setuptools
RUN uv pip install -e .

# Prevent uv from trying to fetch packages at runtime
ENV UV_OFFLINE=1

# generate self-signed certs
RUN make keys
RUN chmod +x /r0b0/scripts/entrypoint.sh
ENTRYPOINT ["/r0b0/scripts/entrypoint.sh"]
