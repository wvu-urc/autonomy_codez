from dronekit import connect
import time

# https://dronekit-python.readthedocs.io/en/latest/guide/vehicle_state_and_parameters.html

# pip install git+https://github.com/dronekit/dronekit-python.git

connection_string = '/dev/ttyACM0'

vehicle = connect(connection_string, wait_ready=True, baud=115200)

for key, value in vehicle.parameters():
    print (" Key:%s Value:%s" % (key,value))


'''
Need to except APIException

'''

attribute_list = {
 "Firmware Versuion": vehicle.version,
 "Autopilot Capabilities" : vehicle.capabilities.ftp,
 "Global Location" : vehicle.location.global_frame,
 "Global Location (relative altitude)" : vehicle.location.global_relative_frame,
 "Local Location (NED)" : vehicle.location.local_frame,
 "Attitude" : vehicle.attitude,
 "Velocity" : vehicle.velocity,
 "GPS" : vehicle.gps_0,
 "Groundspeed" : vehicle.groundspeed,
 "Airspeed" : vehicle.airspeed,
 "Gimbal status" : vehicle.gimbal,
 "Battery" : vehicle.battery,
 "EKF OK?" : vehicle.ekf_ok,
 "Last Heartbeat" : vehicle.last_heartbeat,
 "Rangefinder" : vehicle.rangefinder,
 "Rangefinder distance" : vehicle.rangefinder.distance,
 "Rangefinder voltage" : vehicle.rangefinder.voltage,
 "Heading" : vehicle.heading,
 "Is Armable?" : vehicle.is_armable,
 "System status" : vehicle.system_status,
 "Mode" : vehicle.mode.name,    
 "Armed" : vehicle.armed,

}

while True:
    try:
        for name, attr in attribute_list.items():
            print(f"{name}: {attr}")

        print(f"")
        time.sleep(0.5)

    except Exception:
        vehicle.close()
    finally:
        vehicle.close()