FROM ubuntu:18.04

ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

ENV PYTHONUNBUFFERED 1
ENV PYTHONDONTWRITEBYTECODE 1
COPY requirements.txt .

RUN apt-get update -y
RUN apt-get upgrade -y
RUN apt-get install build-essential git -y
RUN apt install python3-pip -y  \
    && pip3 install setuptools \
    && pip3 install --upgrade pip \
    && pip3 install wheel \
    && pip install -r requirements.txt