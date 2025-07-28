FROM ros:humble-ros-base
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/


COPY . /r0b0
WORKDIR /r0b0
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"
RUN uv sync
