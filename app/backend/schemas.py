from marshmallow import Schema, fields

class RobotControllerSchema(Schema):
    name = fields.Str(required=True)
    controller_name = fields.Str(required=True)
    status = fields.Int()