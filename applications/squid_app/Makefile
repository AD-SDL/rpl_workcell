################################################################################
# Squid App Makefile
################################################################################
APP_MAKEFILE := $(lastword $(MAKEFILE_LIST))
APP_MAKEFILE_DIR := $(dir $(APP_MAKEFILE))
# Need to assign this early to avoid being set by the lab-wide Makefile
COMPOSE_FILE := $(abspath $(APP_MAKEFILE_DIR))/squid_app.compose.yaml

include $(APP_MAKEFILE_DIR)../../Makefile # Lab-wide Makefile and corresponding config
################################################################################
# Variables: Environment Configuration goes here

################################################################################
# Rules: Add anything you want to be able to run with `make <target>` below

run: init # Run the Squid App
	@echo "Starting Squid App"
	@docker-compose -f $(APP_COMPOSE_FILE) up -d squid_app

exec: init # Opens a shell in the Squid App container
	@echo "Opening shell in Squid App container"
	@docker-compose -f $(APP_COMPOSE_FILE) exec -u app squid_app /bin/bash

################################################################################
# Determine which rules don't correspond to actual files (add rules to NOT_PHONY to exclude)
.PHONY: $(filter-out $(NOT_PHONY), $(RULES))
