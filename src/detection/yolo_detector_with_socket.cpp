#include <ros/ros.h>
#include <iostream>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>
#include "px4_application/BoundingBoxes.h"

const char *socket_file = "/home/p520c/Documents/yolov5/unix_socket";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yolo_detector");
    ros::NodeHandle nh;
    
    ros::Publisher yolo_drone_pub = nh.advertise<px4_application::BoundingBoxes>("yolo_detector/bounding_boxes", 5);

    int socket_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if(socket_fd == -1)
    {
        perror("socket");
        exit(1);
    }
    
    struct sockaddr_un server_un;
    bzero(&server_un, sizeof(server_un));
    strcpy(server_un.sun_path, socket_file);
    server_un.sun_family = AF_UNIX;
    if(connect(socket_fd, (struct sockaddr*)&server_un, sizeof(server_un)) == -1)
    {
        perror("connect");
        exit(1);
    }

    px4_application::BoundingBoxes total_boxes;
    // ['y' 目标数 [目标编号 x1 y1 x2 y2 conf cls ...]]
    while(true)
    {   
        char frame_head[4] = {'\0'};
        if(read(socket_fd, frame_head, 1) != 1) // ['y'] 1
        {
            perror("read");
            exit(1);
        }
        
        if(frame_head[0] == 'y')
        {
            if(read(socket_fd, frame_head + 1, 4) != 4) // [{:03d} ] 4
            {
                std::cout << "frame_head error, reparse..." << std::endl;
                continue;
            }
            int target_cnt = std::stoi(frame_head + 1);
            std::cout << "单张图片的目标数量：" << target_cnt << std::endl;
            
            total_boxes.bounding_boxes.clear();
            total_boxes.header.stamp = ros::Time::now();
            for(int i = 0; i < target_cnt; i++)
            {
                char frame_body[28] = {'\0'};
                if(read(socket_fd, frame_body, 28) != 28) // [{:03d} {:03d} {:03d} {:03d} {:03d} {:03d} {:03d} ] 28
                {
                    std::cout << "frame_body error, reparse..." << std::endl;
                    continue;
                }
                int target_id = std::stoi(frame_body, 0);
                int x1 = std::stoi(frame_body + 4);
                int y1 = std::stoi(frame_body + 8);
                int x2 = std::stoi(frame_body + 12);
                int y2 = std::stoi(frame_body + 16);
                int conf = std::stoi(frame_body + 20);
                int cls = std::stoi(frame_body + 24);
                px4_application::BoundingBox single_box;

                single_box.id = target_id;
                single_box.xmin = x1, single_box.ymin = y1, single_box.xmax = x2, single_box.ymax = y2;
                single_box.conf = conf;
                single_box.cls = cls;

                total_boxes.bounding_boxes.push_back(single_box);
                yolo_drone_pub.publish(total_boxes);
                // std::cout << "target_id: " << target_id << "************" << std::endl;
                // std::cout << "box: " << x1 << " " << y1 << " " << x2 << " " << y2 << std::endl;
                // std::cout << "conf: " << conf << std::endl;
                // std::cout << "cls: " << cls << std::endl;
            }
            std::cout << "----------------------------" << std::endl;
        }
        else if(frame_head[0] == 'n')
        {
            total_boxes.bounding_boxes.clear();
            total_boxes.header.stamp = ros::Time::now();
            
            std::cout << "no target" << std::endl;
        }
        else
        {
            total_boxes.bounding_boxes.clear();
            total_boxes.header.stamp = ros::Time::now();
            
            std::cout << "discard" << std::endl;
        }
        yolo_drone_pub.publish(total_boxes);
    }
    close(socket_fd);
    return 0;
}