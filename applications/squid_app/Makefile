################################################################################
# Squid App Makefile
################################################################################
APP_MAKEFILE := $(lastword $(MAKEFILE_LIST))
APP_MAKEFILE_DIR := $(dir $(APP_MAKEFILE))

################################################################################
# Variables: Environment Configuration goes here

PROFILES += squid_app

################################################################################
# Include the Lab-wide Makefile and corresponding config
include $(APP_MAKEFILE_DIR)../../Makefile # Lab-wide Makefile and corresponding config
################################################################################
# Rules: Add anything you want to be able to run with `make <target>` below

################################################################################
# Determine which rules don't correspond to actual files (add rules to NOT_PHONY to exclude)
.PHONY: $(filter-out $(NOT_PHONY), $(RULES))
