#!/usr/bin/env python3

from Iris import Iris 
from threading import Thread


if __name__ == "__main__":
    
    num_of_drones = 3
    drones = []
    for i in range(3):
        drones.append(Iris(i))

    Thread(target = drones[0].draw_square, args=(5,)).start()
    Thread(target = drones[1].draw_square, args=(5,)).start()
    Thread(target = drones[2].draw_square, args=(5,)).start()
    
    
