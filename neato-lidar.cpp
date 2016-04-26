#include <iostream>
#include <cassert>
#include "serial.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

unsigned int neato_checksum(unsigned char *packet_buffer)
{
    // A temporary for the checksum
    unsigned int checksum = 0;
    
    // Build the checksum
    for(int i = 0; i < 10; i++)
    {
        checksum = (checksum << 1) + (packet_buffer[2*i] + (packet_buffer[2*i+1] << 8));
    }
    
    // Fix stuff
    checksum = (checksum & 0x7FFF) + (checksum >> 15);
    checksum = (checksum & 0x7FFF);
    
    // Return the checksum
    return checksum;
}

int main (int argc, char **argv)
{
    // Create a buffer to hold a data packet from the LIDAR
    unsigned char packet_buffer[22];

    // Open a connection to the LIDAR unit
    kybernetes::io::SerialDevice *device = new kybernetes::io::SerialDevice("/dev/ttyUSB0", B115200);
    
    // Synchronize with the LIDAR
    while(1)
    {
        // Get a 22 byte packet from the LIDAR
        device->read((char *) packet_buffer, 22);
        
        // Fetch the checksum
        unsigned int checksum = packet_buffer[20] + (packet_buffer[21] << 8); 
        
        // Is the starting byte correct
        if(packet_buffer[0] == 0xFA && checksum == neato_checksum(packet_buffer))
        {
            // Synchronized
            break;
        }
        
        // Read one byte to offset packet
        device->read((char *) packet_buffer, 1);
    }
    
    // Wait for the packet to be F9 (last packet)
    while(1)
    {
        device->read((char *) packet_buffer, 22);
        if(packet_buffer[1] == 0xF9) break;
    }
     
    // We are synchronized 
    std::cerr << "LIDAR Synchronized" << std::endl;
    
    // Image to show lidar data
    cv::Mat lidar_data(821, 821, CV_8UC1);
    assert(lidar_data.data != NULL);
    cv::namedWindow("lidar", CV_WINDOW_AUTOSIZE);
    
    // Read data
    while (1)
    {
        lidar_data = cv::Scalar(0);
        
        // Loop through all scan lines
        double t = (double) cv::getTickCount();
        for(int i = 0; i < 90; i++)
        {
            // Get a 22 byte packet from the LIDAR
            device->read((char *) packet_buffer, 22);
            
            // Get the RPM (cut of fractional part)
            unsigned short rpm = (packet_buffer[2] + (packet_buffer[3] << 8)) >> 6;
            
            // Loop through the base readings
            for(int j = 0; j < 4; j++)
            {
                // Get the first reading
                unsigned short distance = packet_buffer[(j*2) + 4] + ((packet_buffer[(j*2) + 5] & 0x3F) << 8);
                distance = distance / 15;
            
                // Get this distance
                if(!(packet_buffer[(j*2) + 5] & 0x80) && distance <= 410)
                {
                    // Calculate the angle
                    float bearing = (i * 4) + j;
                    bearing = bearing * (M_PI / 180.0f);
                    
                    // Project this into our space here
                    int x = (cos(bearing) * distance) + 410;
                    int y = (sin(bearing) * distance) + 410;
                    
                    // Draw dot
                    lidar_data.at<unsigned char>(x,y) = 255;
                }
            }
        }
        
        // Show the image
        std::cout << "Scans/s = " << 1.0 / (((double) cv::getTickCount() - t) / cv::getTickFrequency()) << std::endl;
        cv::imshow("lidar", lidar_data);
        cv::waitKey(10);
    }

    return 0;
}
