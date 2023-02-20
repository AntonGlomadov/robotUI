
#find all files with png extension in directory ./images
#print their names and sizes
import os


files = [f for f in os.listdir("./images") if f.endswith(".png")]
for f in files:
	print(f, os.path.getsize("./images/"+f))
