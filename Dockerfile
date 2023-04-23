# syntax=docker/dockerfile:1
# FROM ubuntu:20.04
FROM python:3.10-buster
COPY . /r0b0

# install with versions
WORKDIR /r0b0
RUN pwd
# RUN xargs --help
RUN cat requirements.txt | \
    xargs -n 1 pip3 install
# retry without versions
# to catch pkgs that could not install due to version mismatch
RUN cat requirements.txt | \
    awk -F= '{print $1}' | \
    xargs -n 1 pip3 install

# install ngrok
RUN curl -s https://ngrok-agent.s3.amazonaws.com/ngrok.asc | sudo tee /etc/apt/trusted.gpg.d/ngrok.asc >/dev/null && echo "deb https://ngrok-agent.s3.amazonaws.com buster main" | sudo tee /etc/apt/sources.list.d/ngrok.list && sudo apt update && sudo apt install ngrok