FROM ghcr.io/ad-sdl/wei

# User Configuration
ARG USER_ID=9999
ARG GROUP_ID=9999
ARG CONTAINER_USER=app
USER ${CONTAINER_USER}
WORKDIR /home/${CONTAINER_USER}

# Add python packages to path
ENV PATH="$PATH:/home/${CONTAINER_USER}/.local/bin"

######################################
# App specific changes go below here #
######################################

RUN mkdir -p squid_app
COPY --chown=${USER_ID}:${GROUP_ID} . ./squid_app

# We run our pip installs with cache to speed up subsequent builds,
# this is optional but recommended for developer sanity
RUN --mount=type=cache,target=/home/${CONTAINER_USER}/.cache,uid=${USER_ID},gid=${GROUP_ID} \
    pip install -e squid_app

######################################
# Switch back to root for our entrypoint script,
# which changes container user to match host user.
# This greatly simplifies mounted volume permissions.
USER root
