
'''
while have_a_goal_location

    did i get a return request signal

        yes
            
            update goal_location
        no
            

    did i get a teleop request signal

        yes
            exit
        no

            am i at the goal location

                yes

                    stop, lights
                    break, run loop
                
                no

                    what is my location
                    apply controls to go to location
                    break, run loop


x minutes will be determined by the euclidean distance between the 

'''



def main():
    while True:
        pass