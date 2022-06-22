SHELL := /bin/bash

init:
	python3 ./setup/createEnv.py -y
	source covee_env/bin/activate -y && \
	pip install --upgrade pip && \
	pip install -r ./setup/requirements.txt
clean:
	sudo rm -R -f covee_env
	rm -R -f __pycache__
	rm -R -f covee_env.egg-info