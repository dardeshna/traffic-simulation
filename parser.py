import math
import numpy as np
import os
import csv

output = open("calculated.csv", "w")
writer = csv.writer(output)

for file in os.listdir(os.path.dirname(os.path.abspath(__file__))):
    filename = os.fsdecode(file)
    if filename.endswith(".csv") and filename != "calculated.csv":
        with open(file, 'r') as open_file:
            reader = csv.reader(open_file)
            data = []
            for row in reader:
                data.append(row)
            data = np.array((data[295:-1])[:900])[:,:-1]
            print(len(data))
            
            data = data.astype(np.float)
            print(data)
            line = np.concatenate([np.array([filename]), np.mean(data, axis=0), np.std(data, axis=0)])
            writer.writerow(line)
            