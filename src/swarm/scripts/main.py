#!/usr/bin/env python3

from Iris import Iris 
from threading import Thread


if __name__ == "__main__":
    drone1 = Iris(1)
    drone0 = Iris(0)

    Thread(target = drone0.draw_square(5)).start()
    Thread(target = drone1.draw_square(5)).start()