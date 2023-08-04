from flask.views import MethodView
from flask_smorest import Blueprint, abort

from schemas import *

supported_robots = [{'name': 'UR3e','controller_name': 'ur3e_controller', 'status': 0},
                    {'name': 'UR3','controller_name': 'ur3_controller', 'status': 0},
                    {'name': "Hans Cute",'controller_name': 'hans_controller', 'status': 0},
                    {'name': "Dobot Magician",'controller_name': 'db_controller', 'status': 0}]

robot_blp = Blueprint("Robot", "robot",
                      url_prefix="/robot",
                      description="Operations on robot controllers")


@robot_blp.route("/")
class RunningController(MethodView):
    @robot_blp.response(200, RobotControllerSchema())
    def get(self):
        currently_running_controller = {}
        return currently_running_controller
    
@robot_blp.route("/<string:controller_name>")
class ControllerByName(MethodView):
    @robot_blp.response(200, RobotControllerSchema)
    def get(self, controller_name):
        return supported_robots[0]
