from PIL import Image

im = Image.open("test_image3.jpg")

pix = im.load()

w, h = im.size
value_list = []
for y in range(h):
    for x in range(w):
        value = (round(pix[x,y][0]*31 / 255) << 11) + (round(pix[x,y][1]*63/255) << 5) + (round(pix[x,y][2]*31/255))
        value_list.append(str(value))

print(len(value_list))
with open("ImageOut.h", 'w') as f:
    f.write("static uint16_t ImageData[] = {" + ",".join(value_list) + '};')
        