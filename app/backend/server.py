from flask import Flask
from flask_smorest import Api

from handle import robot_blp as RobotControllerBlueprint

app = Flask(__name__)

app.config["API_TITLE"] = "Robotics Support REST API"
app.config["API_VERSION"] = "v1"
app.config["OPENAPI_VERSION"] = "3.0.3"

api = Api(app)
api.register_blueprint(RobotControllerBlueprint)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080, debug=True)