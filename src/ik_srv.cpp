/*
 * This node uses the ik_solver class to calculate each angle based on a chess move
 * It communicates via FTP with laptop to receive chessMoves from the chessBoard GUI
 * Acts as server with the servoRobot and servoGripper nodes and sends angles
 * @Nikhil Arayath
 * Last Modified: 2/28/21
 */

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <unistd.h>
#include <fstream>
#include "chessRobot/angles_msg.h"
#include "chessRobot/gripper_msg.h"
#include "include/ik_solver.h"

using namespace std;

//initializing clients
ros::ServiceClient client;
ros::ServiceClient grip_client;

//initializing servers
chessRobot::angles_msg srv;
chessRobot::gripper_msg grip_srv;
  
/*
 * This method takes it an x, y, and z, position as well as the gripper response and if setup move
 * Runs the ik_solver with the x y and z to calculate angle for position
 * Moves 3 times down and 3 times up to ensure other pieces are not hit
 * Moves only once if on setup move
 */
void movePiece(double x, double y, double z, int grip_action, bool setup)
{
        //creates new ik_solver
        IkSolver *ik_solver = new IkSolver();
        
        //response if on setup move, only moves to specified position once
        if(setup == true)
        {
            //adds ik solutions to Angles vector
            vector<Angles> angles_vector = ik_solver->ik_solve(x, y, z);
        
            //checks if ik_solver gave values meaning position is reachable
            if(angles_vector.size() > 0)
            {
                Angles angles = angles_vector.at(0);
                
                //Reads from Angles vector and sends to client request
                srv.request.swivel_ang = angles.swivel_angle;
                srv.request.shoulder_ang = angles.shoulder_angle;
                srv.request.elbow_ang = angles.elbow_angle;
                srv.request.wrist_ang = angles.wrist_angle;
                
                //callls client to ensure connection before sending
                if(client.call(srv))
                    {
                        if(srv.response.pos_ok)
                        {
                        }
                        else
                        {
                            cout << "something went wrong";
                        }
                    }
                    else
                    {
                        ROS_ERROR("Failed to call service servoRobot 1");
                    }
            }
        }

        //Not setup move therefore requires multiple motions to not hit piece
        else
        {
        //Brings arm down
            for(int i = 0; i < 3; i++ )
            {
                vector<Angles> angles_vector = ik_solver->ik_solve(x, y, z);
            
                if(angles_vector.size() > 0)
                {
                    Angles angles = angles_vector.at(0);
                    
                    srv.request.swivel_ang = angles.swivel_angle;
                    srv.request.shoulder_ang = angles.shoulder_angle;
                    srv.request.elbow_ang = angles.elbow_angle;
                    srv.request.wrist_ang = angles.wrist_angle;
                    
                    //Moves gripper on 3rd motion when arm is at lowest, opening or closing depewnding on situation
                    if(i == 2)
                    {
                        grip_srv.request.grip_action = grip_action;
                    }
                    else
                    {
                        grip_srv.request.grip_action = 0;
                    }
                    
                    //establishes communication with client
                    if(client.call(srv))
                    {
                        if(srv.response.pos_ok)
                        {
                        }
                        else
                        {
                            cout << "something went wrong";
                        }
                    }
                    else
                    {
                        ROS_ERROR("Failed to call service servoRobot 1");
                    }
                    
                    //delays sending message to gripper client to give time to reach bottom position
                    sleep(1);
                    
                    //connects and send to grip client
                    if(grip_client.call(grip_srv))
                    {
                        if(grip_srv.response.pos_ok)
                        {
                        }
                        else
                        {
                            cout << "something went wrong";
                        }
                    }
                    else
                    {
                        ROS_ERROR("Failed to call service servoGripper 1");
                    }
                    
                }
                //reduces z position so when rerun, lowers the arm
                z = z - 0.1;
            }
            
            sleep(2);
            //increases z back up one to eliminate extra descent after third time through loop
            z = z + 0.1;
            
            //Brings arm back up
            for(int i = 0; i < 3; i++)
            {
                vector<Angles> angles_vector = ik_solver->ik_solve(x, y, z);
            
                if(angles_vector.size() > 0)
                {
                    Angles angles = angles_vector.at(0);
                        
                    srv.request.swivel_ang = angles.swivel_angle;
                    srv.request.shoulder_ang = angles.shoulder_angle;
                    srv.request.elbow_ang = angles.elbow_angle;
                    srv.request.wrist_ang = angles.wrist_angle;
                    
                    grip_srv.request.grip_action = 0;
                
                    if(grip_client.call(grip_srv))
                    {
                        if(grip_srv.response.pos_ok)
                        {
                        }
                        else
                        {
                            cout << "something went wrong";
                        }
                    }
                    else
                    {
                        ROS_ERROR("Failed to call service servoGripper 2");
                    }
                        
                    sleep(1);
                    
                    if(client.call(srv))
                    {
                        if(srv.response.pos_ok)
                        {
                        }
                        else
                        {
                            cout << "something went wrong";
                        }
                    }
                    else
                    {
                        ROS_ERROR("Failed to call service servoRobot 2");
                    }
                }
                //Moves arm upwards when picking up or releasing piece
                z = z + 0.1;
            }
        }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ik_srv");   
    ros::NodeHandle n;    
    client = n.serviceClient<chessRobot::angles_msg>("servoRobot");
    grip_client = n.serviceClient<chessRobot::gripper_msg>("servoGripper");

  
    string chess_move;
    char y_val;
    string x_val;
    string piece;
    string capture_piece;
    
    double x;
    double y;
    double z;
    int grip_action;

    ifstream inFile;
    
    bool setup = true;
    
    while(ros::ok())
    {
        if(setup)
        {
            //setup chess move which is off the board
            chess_move = "k5";
            //takes chess_move stringand parses it for x direction
            x_val = chess_move.substr(1,1);
            //takes 0th element of chess_move for y value
            y_val = chess_move[0];
            //converts to a number value and adds appropriate sizing of board with size of each square and borders
            //Also uses character a to subtract and convert k- a into a double
            y = 0.245745 - 0.05461 * (y_val - 'a' + 1);
            //converts x_value string into a double and adds borders and squre size
            x = 0.102305 + 0.05461 * (stod(x_val));
            
            z = 0.15;
            //z = z + (0.005 * (stod(x_val)));
            //calls move piece method to move to k5
            movePiece(x, y, z, 1, true);
            
            setup = false;
        }
        
        z = 0.35;
        //reads from file sent by chessGui with chess move
        inFile.open("/home/ros/ros_ws/chessRobotMoves/chessMove.txt");
        
        //chechks if file is there
        if(inFile.good())
        {
            //Takes content of file and puts into chess move
            while (inFile >> chess_move) 
            {
            }
                
            inFile.close();
            
            //takes substrings of chess move according to format of first letter the piece, last letter captured piece
            piece = chess_move.substr(0,1);
            capture_piece = chess_move.substr(4,1);
                
            //removes the capture piece before moving moved piece so there are not two pieces on one square
            //E means no piece captured
            if(capture_piece != "E")
            {
                //changes height depending on piece height
                if(capture_piece == "R")
                {
                    z = 0.36;
                }
                else if(capture_piece == "N")
                {
                    z = 0.36;
                }
                else if(capture_piece == "P")
                {
                    z = 0.35;
                }
                else if(capture_piece == "B")
                {
                    z = 0.37;
                }
                else if(capture_piece == "Q")
                {
                    z = 0.36;
                }
                else if(capture_piece == "K")
                {
                    z = 0.36;
                }
                
                //finds x and y value by taking parts of chess move message
                x_val = chess_move.substr(6,1);
                y_val = chess_move[5];
                            
                //converts to doubles according to dimensions of board
                y = 0.245745 - 0.05461 * (y_val - 'a' + 1);
                x = 0.102305 + 0.05461 * (stod(x_val));
                z = z + (0.005 * (stod(x_val)));
                
                //grabs piece as 2 indicates grab
                movePiece(x, y, z, 2, false); 
                    
                //specified place for removing pieces to
                x_val = "5";
                y_val = 'l';
                    
                y = 0.245745 - 0.05461 * (y_val - 'a' + 1);
                x = 0.102305 + 0.05461 * (stod(x_val));
                
                //releases piece shown by 1 at position j5 off the board
                movePiece(x, y, z, 1, false);
            }    
            
            //changes height according to piece
            if(piece == "R")
            {
                z = 0.36;
            }
            else if(piece == "P")
            {
                z = 0.35;
            }
            else if(piece == "N")
            {
                z = 0.36;
            }
            else if(piece == "B")
            {
                z = 0.37;
            }
            else if(piece == "Q")
            {
                z = 0.36;
            }
            else if(piece == "K")
            {
                z = 0.36;
            }
            
            x_val = chess_move.substr(2,1);
            y_val = chess_move[1];
                        
            y = 0.245745 - 0.05461 * (y_val - 'a' + 1);
            x = 0.102305 + 0.05461 * (stod(x_val));
                    
            movePiece(x, y, z, 2, false);
            
            if(capture_piece == "R")
            {
                z = 0.36;
            }
            else if(capture_piece == "N")
            {
                z = 0.36;
            }
            else if(capture_piece == "P")
            {
                z = 0.35;
            }
            else if(capture_piece == "B")
            {
                z = 0.37;
            }
            else if(capture_piece == "Q")
            {
                z = 0.36;
            }
            else if(capture_piece == "K")
            {
                z = 0.36;
            }
                
            x_val = chess_move.substr(6,1);
            y_val = chess_move[5];
                    
            y = 0.245745 - 0.05461 * (y_val - 'a' + 1);
            x = 0.102305 + 0.05461 * (stod(x_val));
                    
            movePiece(x, y, z, 1, false);
            
            //deletes the file to allow for new move
            remove("/home/ros/ros_ws/chessRobotMoves/chessMove.txt");
        }
        
        sleep(3);
    }
    
    return 0;
}


 
