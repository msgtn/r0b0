FROM ros:jazzy-ros-base
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/


COPY . /r0b0
WORKDIR /r0b0
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"
RUN uv sync
RUN uv pip install setuptools
RUN chmod +x /r0b0/scripts/entrypoint.sh
ENTRYPOINT ["/r0b0/scripts/entrypoint.sh"]
