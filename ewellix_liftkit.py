import telnetlib

# Configuration
HOST = "192.168.1.100"
PORT = 50001
COMMAND = "moveTo_absolutePosition,0\n"

# Create a Telnet object
tn = telnetlib.Telnet(HOST, PORT)

try:
    # Send the command
    tn.write(COMMAND.encode('ascii'))

    # Read the response from the device
    response = tn.read_until(b"moveTo_absolutePosition,OK\n", timeout=5)

    # Decode the response to string
    response_str = response.decode('ascii').strip()

    # Print the response
    print("Response from device:", response_str)

finally:
    # Close the connection
    tn.close()
