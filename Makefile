.PHONY: setup test build lint-manifest clean

setup:
	./setup.sh

test:
	.venv/bin/python -m pytest -q

build:
	./build.sh

lint-manifest:
	viam module lint-manifest

clean:
	rm -rf build dist .pytest_cache
