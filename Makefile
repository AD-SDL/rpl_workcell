################################################################################
# RPL Laboratory Makefile
################################################################################
MAKEFILE := $(lastword $(MAKEFILE_LIST))
MAKEFILE_DIR := $(dir $(MAKEFILE))
INCLUDE_DIR := $(MAKEFILE_DIR)/make

include $(INCLUDE_DIR)/boilerplate.mk # Boilerplate rules
include $(INCLUDE_DIR)/lab_config.mk # Laboratory-wide configuration
include $(INCLUDE_DIR)/docker.mk # Docker-related rules

################################################################################
# Rules: Add anything you want to be able to run with `make <target>` below

checks: # Runs all the pre-commit checks
	@cd "$(REPO_DIR)" && pre-commit install && \
		pre-commit run --all-files || { echo "Checking fixes\n" ; pre-commit run --all-files; }

register_diaspora: # Registers diaspora for logging events
	docker compose -f $(LAB_COMPOSE_FILE) run lab_terminal \
		wei/scripts/register_diaspora.py

################################################################################

# Determine which rules don't correspond to actual files (add rules to NOT_PHONY to exclude)
.PHONY: $(filter-out $(NOT_PHONY), $(RULES))
