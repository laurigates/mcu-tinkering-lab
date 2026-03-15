# Thin Makefile wrapper — delegates all targets to just.
# This exists so users who reflexively type `make <target>` still get the right behavior.
# The canonical task runner for this project is `just`. See the justfile for all targets.

.DEFAULT_GOAL := help

%:
	@just $@
