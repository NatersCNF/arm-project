import sys
import numpy as np

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QTimer
import pyqtgraph.opengl as gl


app = QApplication(sys.argv)

view = gl.GLViewWidget()
view.show()
view.setWindowTitle("3D Point Drawer")
view.setCameraPosition(distance=20)


grid = gl.GLGridItem()
grid.scale(1, 1, 1)
view.addItem(grid)


points = []


scatter = gl.GLScatterPlotItem(
    pos=np.empty((0, 3)),
    size=10,
    color=(1, 0, 0, 1)
)

view.addItem(scatter)


line = gl.GLLinePlotItem(
    pos=np.empty((0, 3)),
    color=(0, 1, 1, 1),
    width=2,
    antialias=True
)

view.addItem(line)


def update_plot():
    if len(points) == 0:
        return

    pos = np.array(points)

    scatter.setData(pos=pos)
    line.setData(pos=pos)



def input_loop():
    print("\nEnter point as: x y z")
    print("Example: 3 2 5")
    print("Type 'q' to quit.\n")

    while True:
        user = input("Point: ")

        if user.lower() == 'q':
            break

        try:
            x, y, z = map(float, user.split())

            points.append([x, y, z])

            update_plot()

            print(f"Added point: ({x}, {y}, {z})")

        except:
            print("Invalid input")



QTimer.singleShot(100, input_loop)


sys.exit(app.exec())