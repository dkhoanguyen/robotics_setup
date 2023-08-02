import roslibpy

client = roslibpy.Ros(host='150.22.0.40', port=9090)
client.run()

service = roslibpy.Service(client, '/onrobot_rg/set_command', 'onrobot_rg_control/SetCommand')
request = roslibpy.ServiceRequest()
request['command'] = 'c'
print('Calling service...')
result = service.call(request)

client.terminate()