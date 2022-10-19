import csv 
import random
import math
from xmlrpc.client import FastMarshaller
from scipy.spatial import distance
import numpy as np
import pandas as pd
import copy

class Move(object):
    """docstring for SimulatePedestrian"""
    def __init__(self, anchor1_cord = [2284.5986,968.5631431], anchor2_cord = [2293.396508,967.8745266], 
    noise_low = 0.05, noise_high = 0.1, number_anchors = 2):
    #super(SimulatePedestrian, self).__init__()
    #self.arg = arg
        self.data= []
        self.position=[0,0]
        self.anchor1_cord = anchor1_cord
        self.anchor2_cord = anchor2_cord
        self.noise_low = noise_low
        self.noise_high = noise_high
        self.ped_dist_without_noise = [0, 0] # [dist_from_a1, dist_from_a2]
        self.ped_dist_with_noise = [0, 0]
        self.list_of_peds_in_sim = []	
        self.dist_bet_anchors = 0.
        self.noise = 0.
        self.angles = [0, 0] # angles in radians
        self.pred_coord_from_anchor1 = [0., 0.]
        self.pred_coord_from_anchor2 = [0., 0.]
        self.pred_average_position = [0., 0.]
        self.number_anchors = number_anchors
        self.current_geo_locations = [[0., 0.], [0., 0.], [0., 0.]] #from: anchor1, anchor2, average
        self.ped_ref_coordinates = [0., 0.] # reference coordinate used for computing slopes
        self.current_avg_slope = 0.  #current slope
        self.median_slope = 0. #current median 
        self.heap_slopes = [] #list of 5 slopes
        self.slope_history = [] #has every slope 
        self.slope_lower_bound = 0. #lower bound of the median slope
        self.slope_upper_bound = 0. #upper bound of the medina slope 
        self.ped_out_of_bounds_counter = 0.
        self.ped_out_of_bounds= False
        self.flag_course_correction = False
        self.base_median_slope = 0. #reference median 
        #self.end_correction=True    
        self.current_index = 0
        self.start_correction = False
        self.rate_correction = 0.05
        self.history_positions = []

    
    def getDistance(self,device1):
        dist=distance.euclidean(self.position,device1)
        return dist
    
    def predictAngle(self,dist1,dist2,dist3):
        '''
            dist 1 is the distance between singals (c)
            dist 2 is the distance between pedestrian and the signal they are leaving from (a)
            dist 3 is the distance between pedestrian and the signal they are going towards (b)

            maybe wrong??
       
        '''
        dist1sq=dist1**2
        dist2sq=dist2**2
        dist3sq=dist3**2

        a=((dist1sq+dist2sq)-dist3sq)/(2*dist1*dist2)

        radians=math.acos(a)
        angle=math.degrees(radians)

        
        return angle
     
    def compute_distance_between_anchors(self):
		#pass
        self.dist_bet_anchors = distance.euclidean(self.anchor1_cord,self.anchor2_cord)
    
    def compute_ground_truth_distance_between_ped_and_anchors(self):
		#pass
        #self.ped_dist_without_noise[0] = self.getDistance(self.anchor1_cord)
        #self.ped_dist_without_noise[1] = self.getDistance(self.anchor2_cord)
        self.ped_dist_without_noise[0] = distance.euclidean(self.position, self.anchor1_cord)
        self.ped_dist_without_noise[1] = distance.euclidean(self.position, self.anchor2_cord)

    def compute_distances_with_noise(self, anchor):
		#pass
        self.ped_dist_with_noise[anchor] = self.ped_dist_without_noise[anchor] + self.noise

    def generate_noise(self, _index):
	#pass
        if(self.ped_dist_without_noise[_index] > self.dist_bet_anchors/2):
            self.noise = random.uniform(self.noise_low, self.noise_high)
        else:
            self.noise = random.uniform(0, self.noise_low)
        #print("noise", self.noise)
    

    def add_noise_to_distances(self):
	 	#pass
        #self.compute_ground_truth_distance_between_ped_and_anchors()
        for i in range(self.number_anchors):
            self.generate_noise(i)
            self.compute_distances_with_noise(i)
   
    def compute_angles(self):
		#pass
        for i in range(self.number_anchors):
            distOnePrime = self.ped_dist_with_noise[i]
            distTwoPrime = self.ped_dist_with_noise[(i+1)%2]
            self.angles[i] = math.radians(self.predictAngle(self.dist_bet_anchors,distOnePrime,distTwoPrime))
    

    def predict_new_coordinates_anchor1(self):
		#pass
        self.pred_coord_from_anchor1[0] = self.anchor1_cord[0]+(self.ped_dist_with_noise[0]*math.cos(self.angles[0]))
        self.pred_coord_from_anchor1[1] = self.anchor1_cord[1]+(self.ped_dist_with_noise[0]*math.sin(self.angles[0]))
        
    def predict_new_coordinates_anchor2(self):
        #pass
        self.pred_coord_from_anchor2[0] = self.anchor2_cord[0]+(self.ped_dist_with_noise[1]*math.cos(self.angles[1]))
        self.pred_coord_from_anchor2[1] = self.anchor2_cord[1]+(self.ped_dist_with_noise[1]*math.sin(self.angles[1]))

    def compute_average_location(self):
        #pass
        self.pred_average_position[0] = (self.pred_coord_from_anchor1[0]+self.pred_coord_from_anchor2[0])/2
        self.pred_average_position[1] = (self.pred_coord_from_anchor1[1]+self.pred_coord_from_anchor2[1])/2

    def compute_current_slope(self):
        #pass
        y_diff = self.pred_average_position[1]- self.ped_ref_coordinates[1]
        x_diff = self.pred_average_position[0]- self.ped_ref_coordinates[0]
        if(x_diff>0):
            self.current_avg_slope = y_diff/x_diff

        #print("current_avg_slope", self.current_avg_slope)

    def update_median_slope(self):
        #pass
        self.heap_slopes.append(self.current_avg_slope)
        if(len(self.heap_slopes) > 5):
            self.heap_slopes.pop(0)
            self.median_slope = np.median(self.heap_slopes)
            #print("median_slope", self.median_slope)

    def run_main_sim(self):
        #pass
        print(self.data)
        self.bootstrap()
        self.continue_until_correction_kicks_in()
        #self.correct_pedestrian_path()
        self.finish_the_algorithm()
        '''
        for i in range(5, len(self.data)):
            if(self.flag_course_correction == False):
                self.update_for_no_course_correction(i)
            else:

                self.update_for_course_correction()
            print(self.pred_average_position)
        '''

    def continue_until_correction_kicks_in(self):
        #pass
        
        for self.current_index in range(5, len(self.data)-1):
            #print("current_index", self.current_index)
            self.pred_average_position[0] = self.data.iat[self.current_index,2]
            self.pred_average_position[1] = self.data.iat[self.current_index,3]
            self.position = copy.copy(self.pred_average_position)
            print(self.pred_average_position)
            self.compute_current_slope()
            self.update_median_slope()
            self.update_out_of_bounds_counter()
            #print("ped_counter", self.ped_out_of_bounds_counter)
            if(self.ped_out_of_bounds_counter >= 4):
                #flag = True
                self.start_correction = True
                break

    def finish_the_algorithm(self):
        #pass
        while self.current_index < len(self.data)-1:
            #pass
            self.current_index += 1
            #print("current_index", self.current_index)
            if(self.ped_out_of_bounds_counter == 4):
                self.correct_pedestrian_path()
                self.reset_correction_variables()
            #print("ped_out_of_bounds_counter",self.ped_out_of_bounds_counter)
            self.update_step()
            #self.position = copy.copy(self.pred_average_position)
            print(self.pred_average_position)
            self.update_out_of_bounds_counter()

    def reset_correction_variables(self):
        #pass
        self.ped_out_of_bounds_counter = 0
        self.start_correction = False

    def correct_pedestrian_path(self):
        #pass
        num_steps = int(round((self.median_slope-self.base_median_slope)/self.rate_correction,0))
        count = 0
        #print("ENTER HERE")
        while count <= num_steps:
            #pass
            count += 1
            self.current_index += 1
            #print("current_index", self.current_index)
            #self.adjust_average_coordinates()
            
            #self.pred_average_position = self.position
            self.update_step()
            #self.position = copy.copy(self.pred_average_position)
            print(self.pred_average_position)
            

    def update_step(self):
        #pass
        self.adjust_average_coordinates()
        #self.compute_ground_truth_distance_between_ped_and_anchors()
        #self.add_noise_to_distances()
        #self.compute_angles()
        #self.predict_coordinates()
        self.pred_average_position = copy.copy(self.position)
        self.compute_current_slope()
        self.update_median_slope()
        #self.pred_average_position = self.position


    def update_out_of_bounds_counter(self):
        #pass
        #print("(median, lower, upper)", (self.median_slope, self.slope_lower_bound, self.slope_upper_bound))
        if((self.median_slope < self.slope_lower_bound) or (self.median_slope > self.slope_upper_bound)):
            self.ped_out_of_bounds_counter += 1

    def adjust_average_coordinates(self):
        #pass
        #print("(position, pred_avg)", (self.position, self.pred_average_position))
        self.position[0] = self.position[0]+random.uniform(0, 0.4)
        #print("new_pos, old_pos", self.position[0], self.pred_average_position[0])
        if(self.start_correction==True):            
            self.position[1] = ((self.position[0] - self.ped_ref_coordinates[0])*(self.current_avg_slope+0.05)) + self.ped_ref_coordinates[1] 
        else:
            self.position[1] = ((self.position[0] - self.ped_ref_coordinates[0])*(self.current_avg_slope)) + self.ped_ref_coordinates[1] 
            self.position[1] = self.position[1] + random.uniform(-0.17, 0.25)
        #print("ADJUST, DIFF", self.position, (self.position[0]-self.pred_average_position[0]))

    def bootstrap(self):
        #pass
        self.compute_distance_between_anchors()
        self.ped_ref_coordinates[0] = self.data.iat[self.current_index,2] # avg_x for idx 0
        #print("PRINTING FIRST XCOORD", self.ped_ref_coordinates[0])
        self.ped_ref_coordinates[1] = self.data.iat[self.current_index,3] # avg_y for idx 0
        self.position = copy.copy(self.ped_ref_coordinates)
        #self.history_positions.append(se)
        #self.pred_average_position = self.ped_ref_coordinates
        print(self.position)
        for self.current_index in range(1, 5):
            self.pred_average_position[0] = self.data.iat[self.current_index,2] # avg_x for idx i
            self.pred_average_position[1] = self.data.iat[self.current_index,3]
            self.position = copy.copy(self.pred_average_position)
            self.compute_current_slope()
            self.update_median_slope()
            print(self.pred_average_position)

        self.base_median_slope = self.current_avg_slope
        #self.median_slope = self.base_median_slope
        self.slope_lower_bound=self.base_median_slope-0.1
        self.slope_upper_bound=self.base_median_slope+0.1


    def predict_coordinates(self):
        #pass
        self.predict_new_coordinates_anchor1()
        self.predict_new_coordinates_anchor2()
        self.compute_average_location()
        #self.compute_ped_ref_position()

    def compute_ped_ref_position(self):
        #pass

        self.ped_ref_coordinates[0] = self.pred_average_position[0]
        self.ped_ref_coordinates[1] = self.pred_average_position[1]

 
    def test(self):
        self.position = [2291.1982860000003, 971.0766161023081]
        self.pred_average_position = [2290.898286, 971.2267949999999]
        self.start_correction = True
        self.compute_distance_between_anchors()
        self.adjust_average_coordinates()
        self.compute_ground_truth_distance_between_ped_and_anchors()
        self.add_noise_to_distances()
        self.compute_angles()
        self.predict_coordinates()
        print("position, predicted_coordinates", self.position, self.pred_average_position)
        self.position = [2293.4099620120874, 971.2982026213413]
        self.adjust_average_coordinates()
        #self.compute_distance_between_anchors()
        self.compute_ground_truth_distance_between_ped_and_anchors()
        self.add_noise_to_distances()
        self.compute_angles()
        self.predict_coordinates()
        print("position, predicted_coordinates", self.position, self.pred_average_position)

    def run(self):

        file= pd.read_csv("PedDownNotCorrected2.csv")
        self.data=pd.DataFrame(file)
        self.run_main_sim()



start=Move()
#start.test()
start.run()
#start.position = [2291.1982860000003, 971.0766161023081]
#start.pred_average_position = [2290.898286, 971.2267949999999]















