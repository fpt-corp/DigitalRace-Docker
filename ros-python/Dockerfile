FROM dungpb/dira_ros:ros

RUN apt-get update \
    && apt-get install -y python3 \
    python3-dev \
    python3-pip \
    python \
    python-dev \
    python-pip \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./entrypoint.sh /

WORKDIR /

RUN chmod +x entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

CMD ["/bin/bash"]