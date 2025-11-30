FROM ros:jazzy-ros-base
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/


COPY ./r0b0_interfaces /r0b0/r0b0_interfaces
WORKDIR /r0b0
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

COPY ./pyproject.toml ./Makefile /r0b0/
COPY ./scripts /r0b0/scripts
RUN mkdir -p /r0b0/src/r0b0
RUN uv sync
RUN uv pip install setuptools

COPY ./src/r0b0 /r0b0/src/r0b0
RUN uv sync

COPY ./pages /r0b0/pages

# generate self-signed certs
RUN make keys
RUN chmod +x /r0b0/scripts/entrypoint.sh
ENTRYPOINT ["/r0b0/scripts/entrypoint.sh"]
