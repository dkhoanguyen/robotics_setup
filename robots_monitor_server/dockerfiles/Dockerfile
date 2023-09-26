FROM arm64v8/ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
  && apt-get install -y python3-pip python3-dev python3-dbus\
  && cd /usr/local/bin \
  && ln -s /usr/bin/python3 python \
  && pip3 --no-cache-dir install --upgrade pip \
  && rm -rf /var/lib/apt/lists/*

# set environment variables
ENV PYTHONDONTWRITEBYTECODE 1
ENV PYTHONUNBUFFERED 1

# Dependencies
RUN apt update && \
    apt install -y gcc python3-dev && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* && \
    pip install --upgrade pip

COPY robots_monitor_server/requirements.txt /requirements.txt
RUN pip install -r requirements.txt

# copy project
COPY robots_monitor_server/ /robots_monitor_server/
