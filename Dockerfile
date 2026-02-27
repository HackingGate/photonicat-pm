FROM debian:trixie

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /work/src

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        debhelper \
        devscripts \
        dh-sequence-dkms \
        dpkg-dev \
        lintian \
        make \
    && rm -rf /var/lib/apt/lists/*

COPY . /work/src

CMD ["bash", "-lc", "dpkg-buildpackage -us -uc -b && lintian --allow-root --fail-on error,warning -- /work/*.changes"]
