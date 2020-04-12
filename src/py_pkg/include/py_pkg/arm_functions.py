from __future__ import division
import rospy
import math


arm_values = [1500, 1500, 1500, 1500, 1500, 1500]
coord = [3.1, 3.1, 0]
def update_arm_vals(input_list):
    # uses index 17
    change_waist = ((( (input_list[17] + 32767) * 2000) / 65534 ) - 1000) / 10
    change_x = (-input_list[18]) / 100000
    change_y = (input_list[22]) / 100000
    # /5 makes the the arm move slower ^^^
    
    change_waist = int(change_waist)
    
    arm_values[0] += change_waist
    arm_values[1] += ((( (input_list[18] + 32767) * 2000) / 65534 ) - 1000) / 15
    arm_values[2] += ((( (input_list[22] + 32767) * 2000) / 65534 ) - 1000) / 25
    
    move_by(change_x, change_y)
    
    change_wrist = 0
    if input_list[0] != input_list[4]:
        if input_list[0] == 1:
            change_wrist = -150
        else:
            change_wrist = 150
    
    arm_values[3] += change_wrist
    
    change_rot = 0
    if input_list[3] != input_list[1]:
        if input_list[1] == 1:
            change_rot = -150
        else:
            change_rot = 150
    
    arm_values[4] += change_rot
    arm_values[5] = (( (input_list[19] + 32767) * 1700) / 65534 ) + 1000
    
    
#    for i in range(6):
#        if(arm_values[i] > 2500):
#            arm_values[i] = 2500
#        if(arm_values[i] < 500):
#            arm_values[i] = 500

    #Each servo has its own max angle. The differences cause wrong angles when not handled separately.
    if(arm_values[0] > 2304):  #max angle appears to need 2304, because of the servo documentation
        arm_values[0] = 2304
    if(arm_values[1] > 2304):  #max angle appears to need 2304, because of the servo documentation
        arm_values[1] = 2304
    if(arm_values[2] > 2282):  #max angle appears to need 2304, because of the servo documentation
        arm_values[2] = 2282
    if(arm_values[3] > 2327):  #max angle appears to need 2304, because of the servo documentation
        arm_values[3] = 2327
    if(arm_values[4] > 2500):  #max angle appears to need 2304, because of the servo documentation
        arm_values[4] = 2500
    if(arm_values[5] > 2384):  #max angle appears to need 2304, because of the servo documentation
        arm_values[5] = 2384

    for i in range(6):
        if(arm_values[i] < 500):
            arm_values[i] = 500
    
    
    #rospy.loginfo('%d <-- original', arm_values[0])
    
    return arm_values


def move_by(x, y):
    # Inches: L1 is the lower arm and L2 is the upper arm
    L1 = 5 + (3/4) - (1/32)
    L2 = 7 + (5/16) + (1/32)
    rospy.loginfo('L1: %f -- L2: %f', L1, L2)
    
    #Updates master coordinate
    
    if math.sqrt((x+coord[0])**2 + (y+coord[1])**2) >= 13:
        return
        
    coord[0] += x
    coord[1] += y
    
    # coord[0] could be 0 and you should still be able to move y, but that is not implemented yet
    if coord[0] == 0:
        return
    
    # z is the invisible line between the base and the new coordinate
    z = math.sqrt(coord[0]**2 + coord[1]**2)
    
    #rospy.loginfo('%d / %d', (z**2 + L1**2 - L2**2), (2*z*L1))
    rospy.loginfo('%f / %f', coord[0], coord[1])
    
    # 'A' is the angle inside the triangle at the base   
    A = math.acos((z**2 + L1**2 - L2**2) / (2*z*L1))
    #11, 12
    # shoulder angle
    theta1 = A + math.atan(coord[1]/coord[0]) + 0.192
    #0.192, 0.209
    
    # elbow angle
    #theta2 = math.asin((z/L2) * math.sin(A) + (math.pi/2))
    theta2 = (math.pi + 0.209) - ((math.pi - theta1) - math.asin((coord[1] - (L1 * math.sin(math.pi - theta1))) / L2))
    
    arm_values[1] = int(((theta1 * 2000) / math.pi) + 500)
    arm_values[2] = int(((theta2 * 2000) / math.pi) + 500)
    
    rospy.loginfo('%f %f', theta1, theta2)
    rospy.loginfo('thetaN = %f', math.asin((coord[1] - (L1 * math.sin(math.pi - theta1))) / L2))
    
    


