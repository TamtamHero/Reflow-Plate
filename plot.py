import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

style.use('fivethirtyeight')

fig, ax = plt.subplots()
ax2 = ax.twinx()

ser = serial.Serial('/dev/ttyACM0', 9600)

x_data = []
y1_data = []
y2_data = []
y3_data = []

def animate(i):

  global x_data

  while ser.inWaiting() > 0:
    line = ser.readline().decode('utf-8').strip()
    values = line.split(',')

    duty_cycle = float(values[0])
    temperature = float(values[1])
    target = float(values[2])

    x_data.append(len(x_data)+1)
    y1_data.append(temperature)
    y2_data.append(duty_cycle)
    y3_data.append(target)

  ax.clear()
  ax.set_ylim(0, 270)
  ax2.set_ylim(0, 100)
  ax2.grid(False)
  temp = ax.plot(x_data, y1_data, color='b', label='Temperature')
  target = ax.plot(x_data, y3_data, color='g', label='Target')
  duty = ax2.plot(x_data, y2_data, color='r', label='Duty Cycle')

  # added these three lines
  lns = duty + temp + target
  labs = [l.get_label() for l in lns]
  ax.legend(lns, labs, loc='upper left')

ani = animation.FuncAnimation(fig, animate, interval=1000)
plt.show()