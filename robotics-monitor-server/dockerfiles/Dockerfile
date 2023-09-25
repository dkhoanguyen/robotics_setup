FROM python:3.11-slim-buster

# set work directory
WORKDIR /usr/src/app

# set environment variables
ENV PYTHONDONTWRITEBYTECODE 1
ENV PYTHONUNBUFFERED 1

# Dependencies
RUN apt update && \
    apt install -y gcc python3-dev && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* && \
    pip install --upgrade pip

COPY app/requirements.txt /usr/src/app/requirements.txt
RUN pip install -r requirements.txt

# copy project
COPY app/ /usr/src/app/
