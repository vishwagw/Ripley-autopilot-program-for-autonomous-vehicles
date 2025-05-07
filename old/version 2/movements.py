from pymavlink import mavutil
import numpy as nm

#to start the connection by listening to a UDP port.
the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

#listening to the first heartbeat.

the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))


#send command
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

#recieve and akckonledgement message for arming
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)

print(msg)

#send takoff command
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)

#recieve and akckonledgement message for takeoff
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)

print(msg)

