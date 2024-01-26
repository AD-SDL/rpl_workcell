FROM ghcr.io/ad-sdl/wei

######################################
# App specific changes go below here #
######################################

COPY . ./squid_app

# We run our pip installs with cache to speed up subsequent builds,
# this is optional but recommended for developer sanity
RUN --mount=type=cache,target=/root/.cache \
    pip install -e squid_app


CMD python /home/app/squid_app/src/squid_app.py
######################################
