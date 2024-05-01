import numpy as np
import matplotlib.pyplot as plt 
from scipy.ndimage import gaussian_filter
import ast

#plotting graph 
bg_image = plt.imread('background.png')
height, width, _ = bg_image.shape
print("shape_bg_image", height, width)
heatmap = np.zeros((height,width))

# matrix=[]
with open('heatmap.txt', 'r') as file:
    for line in file:
        sublist = ast.literal_eval(line)

for line in sublist:
    print("cordinate", line[0], line[1])
    scale_x = (line[0] / 1) * width
    scale_y = (line[1] / 1) * height
    print("scale x abd y ", scale_x, scale_y)

    heatmap[abs(int(scale_y)), abs(int(scale_x)) ] += 20
    heatmap = gaussian_filter(heatmap, sigma=5)

_, ax = plt.subplots(figsize=(10, 8))
ax.imshow(bg_image)
ax.imshow(heatmap, cmap="coolwarm", alpha=0.6, extent=[0, bg_image.shape[1], 0,bg_image.shape[0]])
plt.title("Pheromone Intensity")
plt.axis('on')
# plt.show()
plt.savefig("trajectory&pheromone.png")
