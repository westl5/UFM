import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


ser = serial.Serial(
    port='COM3',
    baudrate=9600,
    timeout=1
)

# Initialize the plot
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_xlim(-0, 4095)  # Adjust these limits based on your data range
ax.set_ylim(-0, 4095)
ax.grid(True)
ax.set_title('Real-ish time Object Positions')
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')

# Initialize scatter plots for each object with different colors
scat0 = ax.scatter([], [], c='red', label='Object 0')
scat1 = ax.scatter([], [], c='blue', label='Object 1')
scat2 = ax.scatter([], [], c='green', label='Object 2')
ax.legend()

data_points = {"obj0": (0, 0), "obj1": (0, 0), "obj2": (0, 0)}

def update():
    res_b = ser.readline()
    res = res_b.decode()
    res = res.split(',')
    try:
        res = [int(coord.strip()) for coord in res]
        obj0 = (res[0], res[1])
        obj1 = (res[2], res[3])
        obj2 = (res[4], res[5])
        objs = [obj0, obj1, obj2]
        print(objs)
        return objs
    except:
        pass


def animate(frame):
    data = update()
    if data is not None:
        # Update scatter plot positions
        scat0.set_offsets(data[0])
        scat1.set_offsets(data[1])
        scat2.set_offsets(data[2])
    return scat0, scat1, scat2

# Create animation
anim = FuncAnimation(fig, animate, interval=1, blit=True)

# Show the plot
plt.show()