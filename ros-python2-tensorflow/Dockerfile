FROM dungpb/dira_ros:ros-cuda

RUN apt-get update \
    && apt-get install -y python \
    python-dev \
    python-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip install tensorflow-gpu

# setup entrypoint
COPY ./entrypoint.sh /

WORKDIR /

RUN chmod +x entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

CMD ["/bin/bash"]