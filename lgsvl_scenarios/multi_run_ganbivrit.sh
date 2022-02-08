#!/bin/bash
#	This file is called script1.sh
#	1. Add code into file:
#	#!/bin/bash 
#	2. Make file executable:
#	chmod +x runme.sh
#	3. Run the file from terminal:
#	./script1.sh

#   First argument - number of runs (if numeric), 
#                   else - file of previouse saved permutations
#   Next arguments - permutations to set ( relevant if First argument - numeric)

#     Choose permutations
#     e - for random ego start pos
#     p - for random_object_start_pos
#     t - for random_object_trajectory
#     s - for random_object_speed :""")

#ros2 launch carteav_launch_pkg general_launch.py detection:=false platform:=sim_cart site:=u
#sleep 10

python3 ganbivrit-01-static-bypass.py 1 p
sleep 1

python3 ganbivrit-02-static-roadblock.py 1 p
sleep 1

python3 ganbivrit-03-dynamic_pedestr_90.py 1 s
sleep 1

python3 ganbivrit-04-dynamic_pedestr_45.py 1 s
sleep 1

python3 ganbivrit-05-dynamic-vehicles-on-front.py 1 p s
sleep 1

python3 ganbivrit-06-pedestrian-cross-loop.py 1 p s
sleep 1

# python3 ganbivrit-07-car-cross.py 2 e p t s
# sleep 1




