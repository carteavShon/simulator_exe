import sys
from datetime import datetime

def get_current_datetime_str():
    now = datetime.now()
    dt_string = now.strftime("%Y%m%d_%H%M%S")
    return dt_string 

class PermutationParams():

    scenario_filename = ''   
    
    def __init__(self, scenario_num):
        self.scenario_num = scenario_num
        self.random_ego_start_pos = False
        self.random_object_start_pos = False
        self.random_object_trajectory = False
        self.random_object_speed = False  
        self.loop_count = 1 
        self.source = 'none' # 'random' 'file' 'none'
     
    def set_params(self, params: list):
        if 'e' in params:
            self.source = 'random'
            self.random_ego_start_pos = True
        if 'p' in params:
            self.source = 'random'
            self.random_object_start_pos = True
        if 't' in params:
            self.source = 'random'
            self.random_object_trajectory = True
        if 's' in params:
            self.source = 'random'
            self.random_object_speed = True

    def input_params(self):
        randoms = None

        if (len(sys.argv) > 1):

            if (sys.argv[1].isnumeric()):    
                self.loop_count = int(sys.argv[1])

                print('loop for ' +str(self.loop_count) + ' times')

                args = [sys.argv[i] for i in range(2,len(sys.argv))]

                self.set_params(args)
            else: # filename   
                PermutationParams.scenario_filename = sys.argv[1]    
                randoms = PermutationParams.read_scenario_randoms(PermutationParams.scenario_filename)                
                self.source = 'file'

            return randoms

    def save_scenario_randoms(self, randoms: list):

        PermutationParams.scenario_filename = str(self.scenario_num)  + '-' + get_current_datetime_str() + '.txt'        
          
        print('set randoms: ' + str(randoms))   
        print('save to: ' + PermutationParams.scenario_filename)

        with open(PermutationParams.scenario_filename, 'w') as file:
            [file.write(str(item) + '\n') for item in randoms]
            file.close()

    def read_scenario_randoms(self) -> list:
        randoms = None
        with open(PermutationParams.scenario_filename) as file:
            randoms = [float(line.strip().split('\n')[0]) for line in file]

        print('randoms from file: ' + PermutationParams.scenario_filename + ': ' + str(randoms))

        file.close()
        return randoms
