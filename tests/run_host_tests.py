"""Run production application/driver logic against deterministic HAL substitutes."""
from pathlib import Path
import argparse
import shutil
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('--compiler', default=shutil.which('g++'))
args = parser.parse_args()
if not args.compiler:
    parser.error('Pass --compiler with a host g++ executable')
root = Path(__file__).resolve().parents[1]
tests = root / 'tests'
includes = [tests / 'stubs'] + [root / p for p in [
    'UserCode', 'Modules/MotorDrivers/core', 'Modules/MotorDrivers/controllers',
    'Modules/MotorDrivers/motors/DJI', 'Modules/BasicComponents/bsp/can_driver',
    'Modules/BasicComponents/bsp/gpio_driver', 'Modules/BasicComponents/libs/utils/ring_buffer',
    'Modules/BasicComponents/memory/DMABuffer', 'Modules/BasicComponents/memory/StaticObject',
    'Modules/BasicComponents/libs/control/pid_motor', 'Modules/BasicComponents/libs/utils/crc',
]]
sources = [tests / 'host_tests.cpp'] + [root / p for p in [
    'UserCode/upper_uart.cpp', 'Modules/MotorDrivers/motors/DJI/dji.cpp',
    'Modules/MotorDrivers/controllers/motor_vel_controller.cpp',
    'Modules/MotorDrivers/controllers/motor_pos_controller.cpp',
    'Modules/BasicComponents/libs/control/pid_motor/pid_motor.cpp',
    'Modules/BasicComponents/bsp/can_driver/can_driver.cpp',
]]
with tempfile.TemporaryDirectory(prefix='motorcontrol-host-') as folder:
    for motor in (0, 1):
        binary = Path(folder) / ('tests-%s.exe' % motor)
        command = [args.compiler, '-std=gnu++2a', '-O2', '-Wall', '-Wextra',
                   '-Wno-unused-parameter', '-include', str(tests / 'stubs/host_compat.hpp'),
                   '-DUSE_M2006_MOTOR=%s' % motor]
        command += ['-I' + str(p) for p in includes]
        command += [str(p) for p in sources] + ['-o', str(binary)]
        subprocess.run(command, check=True)
        print('M2006/C610' if motor else 'M3508/C620', flush=True)
        subprocess.run([str(binary)], check=True)
