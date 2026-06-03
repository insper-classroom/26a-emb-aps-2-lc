#!/usr/bin/env python3
# Grabs raw data from the Pico's UART and plots it as received
# Install dependencies:
# python3 -m pip install pyserial matplotlib
# Usage: python3 plotter.py [port]
# eg. python3 plotter.py COM3
# Default port is COM3 (Windows). Override via command line arg.
# see matplotlib animation API for more: https://matplotlib.org/stable/api/animation_api.html
import serial
import sys
import matplotlib
matplotlib.use('TkAgg')  # backend compativel com Windows
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.lines import Line2D

# disable toolbar
plt.rcParams['toolbar'] = 'None'

DEFAULT_PORT = 'COM3'

class Plotter:
    def __init__(self, ax):
        self.ax = ax
        self.maxt = 250
        self.tdata = [0]
        self.ydata = [3.3/2]
        self.line = Line2D(self.tdata, self.ydata)
        self.ax.add_line(self.line)
        self.ax.set_ylim(0, 3.3)
        self.ax.set_xlim(0, self.maxt)

    def update(self, y):
        lastt = self.tdata[-1]
        if lastt - self.tdata[0] >= self.maxt:  # drop old frames
            self.tdata = self.tdata[1:]
            self.ydata = self.ydata[1:]
            self.ax.set_xlim(self.tdata[0], self.tdata[0] + self.maxt)
        t = lastt + 1
        self.tdata.append(t)
        self.ydata.append(y)
        self.line.set_data(self.tdata, self.ydata)
        return self.line,

def serial_getter():
    # grab fresh ADC values
    # note sometimes UART drops chars so we try a max of 5 times
    # to get proper data
    while True:
        for i in range(5):
            line = ser.readline()
            try:
                line = float(line)
            except ValueError:
                continue
            break
        yield line

port = sys.argv[1] if len(sys.argv) >= 2 else DEFAULT_PORT
print(f"Conectando na porta: {port}")

try:
    ser = serial.Serial(port, 115200, timeout=1)
except serial.SerialException as e:
    print(f"Erro ao abrir porta serial '{port}': {e}")
    print("Verifique se o Pico está conectado e a porta está correta.")
    sys.exit(1)

fig, ax = plt.subplots()
plotter = Plotter(ax)
ani = animation.FuncAnimation(fig, plotter.update, serial_getter, interval=1,
                              blit=True, cache_frame_data=False)
ax.set_xlabel("Samples")
ax.set_ylabel("Voltage (V)")
fig.canvas.manager.set_window_title('Microphone ADC example')
fig.tight_layout()
plt.show()