FROM ubuntu:18.04
LABEL author="Edoardo De Din" 
LABEL author_contact="ededin@eonerc.rwth-aachen.de"

ENV PYTHONUNBUFFERED 1
ENV PYTHONDONTWRITEBYTECODE 1
WORKDIR /covee-service
COPY requirements.txt .

RUN apt-get update -y \
    && apt-get upgrade -y \ 
    && apt-get install build-essential -y \
    && apt install python3-pip -y  \
    && pip3 install --upgrade pip \
    && pip3 install -r requirements.txt
