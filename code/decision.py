import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do with given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            if (len(Rover.nav_angles) < Rover.stop_forward)|(Rover.hunt_rock):
                    print("Lack of navigable terrain pixels, all stop")
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'
            #if Rover.vel < 0.1:
            #    Rover.throttle = 0
            #    Rover.brake = 0
            #    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    #print("nav_angles mean: ",np.mean(Rover.nav_angles))
                    #avg_angle = np.mean(Rover.nav_angles)
                    #if np.isfinite(avg_angle):
                    #    if avg_angle < -0.2:
                    #        Rover.steer = -15
                    #    if avg_angle >= -0.2:
                    #        Rover.steer = 15
                    #else:
                    #    Rover.steer = -15
                    Rover.steer = -15
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # if it's a tight turn, rotate until it is not.
                    if (np.absolute(np.mean(Rover.nav_angles)) > 0.8*np.pi/4.0):
                        print("Tight Turn, Rotating")
                        if np.mean(Rover.nav_angles) < 0:
                            Rover.steer = -15
                        if np.mean(Rover.nav_angles) >= 0:
                            Rover.steer = 15
                    else:
                        # Set throttle back to stored value
                        Rover.throttle = Rover.throttle_set
                        # Release the brake
                        Rover.brake = 0
                        # Set steer to mean angle
                        Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                        Rover.mode = 'forward'
                if (Rover.hunt_rock)&(Rover.enable_hunt):
                    Rover.mode = 'hunt'
                                    
        elif Rover.mode == 'hunt':
            # If there is a rock, approach it
            if Rover.rock_angles is not None:
                if (len(Rover.rock_angles) > 0)&(Rover.hunt_rock):
                    if (np.mean(Rover.rock_dists) > Rover.rock_limit):
                        Rover.throttle = 0.2 # slow approach
                        Rover.brake = 0 #release brake
                        Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)                        
                    else:
                        Rover.throttle = 0.0
                        Rover.brake = Rover.brake_set
                        Rover.mode = 'pickup'
            else:
                Rover.mode = 'forward' # If the rover can't see a rock anymore (no rock angles) then continue
                
        elif Rover.mode == 'pickup':
            if Rover.vel > 0.0:
                Rover.throttle = 0.0
                Rover.brake = Rover.brake_set  
                Rover.mode = 'stop'
            if not Rover.near_sample:
                Rover.mode = 'stop'
        
        
            
                
    # If no nav angles, just go forward
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
    
    print("Mode: ",Rover.mode)
    print("Hunt_Rock:",Rover.hunt_rock)
    
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
            
    return Rover

