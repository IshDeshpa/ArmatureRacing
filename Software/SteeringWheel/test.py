from PIL import Image, ImageChops
import os

im = Image.open(r"racetrackHarrisHill.PNG")

width, height = im.size

resizeWidth = 180;
resizeHeight = height*resizeWidth//width

thresh = 255
fn = lambda x : 255 if x < thresh else 0
out = ImageChops.invert(im.resize((resizeWidth, resizeHeight))).convert('L').point(fn, mode='1')

#out = im.resize((resizeWidth, resizeHeight)).convert('1')

os.remove("track.h")
track = open("track.h", "w")
track.write("const uint16_t track [] PROGMEM = {")

#False is black, True is white

currColor= out.getpixel((0, 0)) 
startColor = out.getpixel((0, 0))
currCnt = 0
size = 0

for y in range(0, resizeHeight):
	for x in range(0, resizeWidth):
		if out.getpixel((x, y)) == currColor:
			#track.write("false")
			currCnt += 1
		else:
			#print(currCnt)
			track.write("0x" + format(currCnt, 'x').zfill(4))
			currCnt=1
			currColor = out.getpixel((x, y))
			track.write(", ")
			size += 1
			

track.write("0x" + format(currCnt, 'x').zfill(4))
currCnt=0
size += 1
currColor = out.getpixel((x, y))

track.write("};")

width, height = out.size
print(width, height, size)

track.write("\nconst int width = " + str(width) + ";")
track.write("\nconst int height = " + str(height) + ";")
track.write("\nconst int trackSize = " + str(size) + ";")
track.write("\nconst int startColor = " + str(startColor) + ";")

track.close()


#out.show()