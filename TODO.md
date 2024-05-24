Autonomy Rules
        
    General
        30-60 minutes 
        no futher than 2 km 
        MUST DECLARE WHEN TRAVSERING OUT OF ORDER
        Must displace message of successful completiton on operator display

        WITHOUT PENALTY - Operators may at any point send a signal to the rover to abort the current attempt and autonomously return to the previous GNSS coordinate, post, or object, and stop within 10 m of it.

        STOPPED AT ANY LOCATION - (whether after a successful arrival, an unsuccessful arrival, or from an abort), teams may program the next or previous location and make any changes to the controls, but may not drive the rover changes to the controls, but may not drive the rover.
        
    Locations
        2 GNSS points
            no vivinity given - is just point
            stop within 3 meters
        3 ARUCO posts
            GNSS location in a 5 to 20 meter vicinity
            stop within 2 meters
        2 objects
            GNSS in <10 meter vicinity
            stop within 2 meters
            obstacles in the way of the second object that need to avoid 

    Lights
        Red - autonomous
        Blue - manual driving 
        Flashing green - arrival 
        
    
Code Plan

    Basics
        Get a point
        Know where it is in relation to you
        Apply controls to get to recieved point

        Search can be done manually at this point, just need to REALLY be able to tune the tolerances for stopping

    URC newances
        
        Different types of points - requires a different system to dictate the behavior upon arrival
            
            Search based - Aruco or objects 
                
                Even this part does not need to be automated 
                
                One part holds points to search for, planner just executes and knows of one at a time
                part holding points is constantly waiting to get signal of object being searched having been 
                seen 
            
            GNSS Point based
                
                Just apply the simple planner conrol loop


Code To Do's:
    (ranked by importance)

    - flash lights when at goal in mgmt.py
    - calculate gps location from yolo + aruco 
    - Update goal gps when see aruco or objects

    - Have data structures to manage queueing in mgmt.py
    - Have feedback from controller to send next goals in mgmt.py

    - Add automated search
    - Remove zero point turns

Yolo Notes:

    1 meter minimum range

    2 meters - 5 cm of error, 40 cm out of axis 
    
    7 meters - 50 cm of error 

    11 meters max range 

    


                

