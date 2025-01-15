'''
EE40I6 Capstone Project - UFM - Group 10 
members: westl5, gillg62, xur76, yum77
'''

import numpy as np
#import open3d as o3d
import serial

if __name__ == "__main__":
    f = open("irleds.csv", "w")    #create a new file for writing 
    
    for i in range(1):
        s = serial.Serial('/dev/cu.usbserial-0001', 9600, timeout = 10)
        print("Opening: " + s.name)

        # reset the buffers of the UART port to delete the remaining data in the buffers
        s.reset_output_buffer()
        s.reset_input_buffer()

        # wait for user's signal to start the program
        input("Press Enter to start communication...")
        # send the character 's' to MCU via UART
        # This will signal MCU to start the transmission
        s.write('s'.encode())

        # recieve SCANS measurements from UART of MCU
        for j in range(1000):
            x = s.readline()
            print(x.decode())
            f.write(x.decode())    #write to file as 
        # the encode() and decode() function are needed to convert string to bytes
        # because pyserial library functions work with type "bytes"

        #close the port
        print("Closing: " + s.name)
        s.close()

    f.close()                                 
    '''
    
    #Read the test data in from the file we created        
    print("Read in the prism point cloud data (pcd)")
    pcd = o3d.io.read_point_cloud("halldata2dx.xyz", format="xyz")

    #Lets see what our point cloud data looks like numerically       
    print("The PCD array:")
    print(np.asarray(pcd.points))

    #Lets see what our point cloud data looks like graphically       
    print("Lets visualize the PCD: (spawns seperate interactive window)")
    o3d.visualization.draw_geometries([pcd])

    #Give each vertex a unique number
    yz_slice_vertex = []
    for i in range(0,XMEASUREMENTS*SCANS):
        yz_slice_vertex.append([i])

    #Define coordinates to connect lines in each yz slice        
    lines = []  
    for i in range(0,XMEASUREMENTS*SCANS, SCANS):
        for j in range(0, SCANS):
            if (j==SCANS-1):
                lines.append([yz_slice_vertex[i+j], yz_slice_vertex[i]])
            else:
                lines.append([yz_slice_vertex[i+j], yz_slice_vertex[i+j+1]])
        
    #Define coordinates to connect lines between current and next yz slice        
    for i in range(0,XMEASUREMENTS*SCANS-SCANS, SCANS):
        for j in range(0, SCANS):
            lines.append([yz_slice_vertex[i+j], yz_slice_vertex[i+j+SCANS]])
        
    #This line maps the lines to the 3d coordinate vertices
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

    #Lets see what our point cloud data with lines looks like graphically       
    o3d.visualization.draw_geometries([line_set])
    '''                          
    
 
