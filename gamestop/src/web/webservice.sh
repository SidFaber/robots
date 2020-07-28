#! /bin/bash

cd /opt/web
export FLASK_APP=webservice.py
export FLASK_ENV=development
flask run --host=0.0.0.0 --port 5000
