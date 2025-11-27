import cflib.crtp

# Initiate the low level drivers
cflib.crtp.init_drivers()

print("Scanning interfaces for Crazyflies...")

for a in range(1, 255):
    address = 0xE7E7E7E700 + a
    available = cflib.crtp.scan_interfaces(address)
    print(f"Scanned {address:x}")
    for i in available:
        print(f"Drone {i[0]} available")
