SHELL := /bin/bash

init:
	python3 ./create_env/createEnv.py -y
	source covee_env/bin/activate -y && \
	pip install --upgrade pip
	pip install -r requirements.txt
clean:
	sudo rm -R -f covee_env
	rm -R -f __pycache__
	rm -R -f covee_env.egg-info
	rm -R -f covee.egg-info
	rm -R -f dist
	rm -R -f build