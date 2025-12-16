#!/usr/bin/env python3
import os
import pty
import time
import errno

MASTER_SLAVE_SYMLINK = '/tmp/ttyV0'

master, slave = pty.openpty()
slave_name = os.ttyname(slave)

# Ensure symlink points to the slave pty
try:
    if os.path.islink(MASTER_SLAVE_SYMLINK) or os.path.exists(MASTER_SLAVE_SYMLINK):
        os.remove(MASTER_SLAVE_SYMLINK)
    os.symlink(slave_name, MASTER_SLAVE_SYMLINK)
except OSError as e:
    print(f"Failed to create symlink {MASTER_SLAVE_SYMLINK} -> {slave_name}: {e}")
    raise

print(f"Serial simulator running. Slave device: {slave_name}")
print(f"Symlink created: {MASTER_SLAVE_SYMLINK} -> {slave_name}")

try:
    while True:
        # create a simple test packet: positions then velocities
        packet = "0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0\n"
        try:
            os.write(master, packet.encode('ascii'))
        except OSError as e:
            if e.errno == errno.EIO:
                # Slave closed, break
                print("Slave closed, exiting simulator")
                break
            else:
                raise
        time.sleep(0.05)
except KeyboardInterrupt:
    pass
finally:
    try:
        os.remove(MASTER_SLAVE_SYMLINK)
    except OSError:
        pass
    os.close(master)
    os.close(slave)

print('Serial simulator exiting')
