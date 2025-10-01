import cv2
import numpy as np
import math
from PIL import Image, ImageTk
from queue import Queue

class cost_map:
	def __init__(self,graphics):
		self.graphics = graphics

		self.inflation_radius = 18 # radius of our robot is 18 pixel or cm
		self.graphics.environment.robots[0].set_bot_size(body_cm = 2*self.inflation_radius)

		self.map_width = int(self.graphics.environment.width*self.graphics.scale)
		self.map_height = int(self.graphics.environment.height*self.graphics.scale)
		
		# To adjust the map size, please go to E160_graphics.py line 18, and change the self.scale there.
		# self.scale shall be half of your image size, for example, if your map image is 500 x 500 pixel, set your self.scale = 250
		# The default map size is 500 x 500 pixel. In case you want a smaller size for debugging, you can change the value of self.scale.
		
		try:
			self.load_map(map = "maps/testmap.png") #load map, put your own map here
		except:
			self.graphics.show_map_button.configure(state="disabled")
			print ("no map loaded") #if fail to find the map png
			return
		
		self.show_map()
		self.compute_costmap()
		self.get_vis_map()
		self.save_vis_map(map = "maps/test_vis_map2.png")
		self.save_costmap(file_path= 'maps/test_cost_map2.txt')

	#load occupancy grid into self.map
	#self.map is a numpy 2d array
	#initialize self.costmap, a numpy 2d array, same as self.map
	def load_map(self,map="maps/testmap2.png"):
		self.map_img = Image.open(map).convert('L')
		self.map_img = self.map_img.resize((int(self.map_width),int(self.map_height)))
		# self.graphics.draw_map(map_img=self.map_img)
		self.map = cv2.imread(map,cv2.IMREAD_GRAYSCALE)
		print (self.map.dtype)
		print ("Loaded map dimension: %d x %d pixel"%(self.map.shape[0],self.map.shape[1]))
		self.map = cv2.resize(self.map, dsize=(int(self.map_width),int(self.map_height)), interpolation=cv2.INTER_CUBIC)
		
		self.distmap=np.full_like(self.map, -1, dtype=int) #map for saving distance to the nearest obstacle, initialize with an array with all -1

		self.costmap=np.copy(self.map).astype(float) #the cost map you are going to work on
		self.vis_map=np.copy(self.map) #map for visualization, intialize same as the map

	#save your costmap into a grayscale image
	def save_vis_map(self,map="maps/vis_map.png"):
		save_img = Image.fromarray(self.vis_map)
		save_img.save(map)

	def show_vis_map(self):
		self.vis_map_img=Image.frombytes('L', (self.vis_map.shape[1],self.vis_map.shape[0]), self.vis_map.astype('b').tostring())
		self.graphics.draw_map(map_img=self.vis_map_img)

	#display costmap on the dialogue window, in most cases, the cost map cannot be properly shown
	def show_costmap(self):
		self.costmap_img=Image.frombytes('L', (self.costmap.shape[1],self.costmap.shape[0]), self.costmap.astype('b').tostring())
		self.graphics.draw_map(map_img=self.costmap_img)

	#display binary occupancy grid on the dialogue window 
	def show_map(self):
		self.graphics.draw_map(map_img=self.map_img)

	def save_costmap(self, file_path, fmt='%.3f', delimiter='\t'):
		np.savetxt(file_path, self.costmap, fmt=fmt, delimiter=delimiter)



	def compute_costmap(self):
		'''
		Your main effort is here. At the end of the program, you shall have a costmap with correct inflation
		'''

		#self.costmap is initialized same as the map. That is, a white pixel is 255.0 (free), a black pixel is initalized as 0.0 (occupied). 
		
		#We first let all black obstacles to be 0 on distmap
		self.distmap[self.map<20.0] = 0
		# print("there are: "+str(np.sum(self.distmap==0))+ " pixels contain obstacle")

		#It is up to you how to get distmap, you may choose not to use it even

		#just a demo how you generate a costmap, we believe any obstacle pixel (distvalue = 0) has a cost of 255
		#we also believe all other pixels shall have cost of 128
		self.costmap[self.distmap==0]=255.0
		self.costmap[self.distmap!=0]=128.0
		pass


	#scale costmap to 0 - 255 for visualization
	def get_vis_map(self):
		'''
		get the map for visualization
		'''

		min_val = min(np.min(self.costmap),0) 
		max_val = np.max(self.costmap)

		# Normalize the costmap
		self.vis_map = np.uint8(255 - (self.costmap - min_val) / (max_val - min_val) * 255)
