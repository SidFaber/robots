#! /bin/bash

cd /home/ubuntu/robots_ws/src/robots/gamestop/src
export FLASK_APP=api.py
export FLASK_ENV=development
flask run --host=0.0.0.0 --port 5000
