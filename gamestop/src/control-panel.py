#
#  Start with 
# export FLASK_APP=api.py
# flask run --host=0.0.0.0
#
from flask import Flask
from flask import render_template

app = Flask(__name__)
arm = Arm ("arm-api")

@app.route('/api')
def hello_world():
    return 'Hello, World!'

