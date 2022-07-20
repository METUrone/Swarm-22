#!/usr/bin/env python3
import csv
from posixpath import split
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure(figsize=(8,8))

ax = fig.add_subplot(111, projection='3d')

file = open('./agent0_logs.csv')
csv_reader = csv.reader(file)

header = []
header = next(csv_reader)

print(header)
rows = []

for i in csv_reader:
    rows.append(i)

X = []
Y = []
Z = []

num_of_agents = 0

for i in range(50):
    try:
        file = open('./agent{}_logs.csv'.format(i))
    except FileNotFoundError:
        print(str(i) + "  ----------")
        num_of_agents = i 
        break

for j in range(num_of_agents):
    file = open('./agent{}_logs.csv'.format(j))
    csv_reader = csv.reader(file)   

    header = []
    header = next(csv_reader)   

    print(header)
    rows = []   

    for i in csv_reader:
        rows.append(i)

    for i in range(len(rows)):
        position = rows[i][2].split(", ")
        X.append(float(position[0]))
        Y.append(float(position[1]))
        Z.append(float(position[2]))


    plt.plot(X, Y, Z)
    X = []
    Y = []
    Z = []

plt.show()