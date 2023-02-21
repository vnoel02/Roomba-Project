//Group 2: Victor, Zain, Charles

/**
 * make-plan.cc
 * 
 * Sample code for a robot that has two front bumpers and a laser, and
 * which is provided with localization data.
 *
 * The code also allows the controller to read and write a "plan", a sequence
 * of location that the robot should move to and to read in a "map", a matrix
 * of 1 and 0 values that can be used as an occupancy grid.
 *
 * Written by: Simon Parsons
 * Date:       4th December 2011
 *  
 **/


#include <iostream>
#include <fstream>
#include <libplayerc++/playerc++.h>
#include <list>
#include <algorithm>
#include <vector>
#include <bits/stdc++.h>
using namespace PlayerCc;
using namespace std;  

/**
 * Global constants
 *
 **/

const int SIZE = 32; // The number of squares per side of the occupancy grid
                     // (which we assume to be square)

/**
 * Function headers
 *
 **/

player_pose2d_t readPosition(LocalizeProxy& lp);
void printRobotData(BumperProxy& bp, player_pose2d_t pose);
void printLaserData(LaserProxy& sp);

void readMap(int[SIZE][SIZE]);
void readReverseMap(int[SIZE][SIZE]);
void writeMap(int [SIZE][SIZE], vector<double>);
void writeReverseMap(int [SIZE][SIZE]);
void printMap(int [SIZE][SIZE]);
int  readPlanLength(void);
void readPlan(double *, int);
void printPlan(double * ,int);  
void writePlan(double *, int);

void writeNewPlan(std::vector<double>);

/**
 * main()
 *
 **/

int main(int argc, char *argv[])
{  

  

  // Variables
  int counter = 0;
  double speed;            // How fast do we want the robot to go forwards?
  double turnrate;         // How fast do we want the robot to turn?
  player_pose2d_t  pose;   // For handling localization data


  //Coordinate Array
  

  // The occupancy grid

  int oGrid[SIZE][SIZE];

  // Occupancy grid for map out
  
  int nGrid[SIZE][SIZE];

  // The set of coordinates that makes up the plan

  int pLength;
  double *plan;

  // Set up proxies. These are the names we will use to connect to 
  // the interface to the robot.
  PlayerClient    robot("localhost");  
  BumperProxy     bp(&robot,0);  
  Position2dProxy pp(&robot,0);
  LocalizeProxy   lp (&robot, 0);
  LaserProxy      sp (&robot, 0);
  
  // Allow the program to take charge of the motors (take care now)
  pp.SetMotorEnable(true);

  // Map handling
  //
  // The occupancy grid is a square array of integers, each side of
  // which is SIZE elements, in which each element is either 1 or 0. A   //Note: Map is already given for you, you need to convert 32x32 and 16x16 so you can read the map
  // 1 indicates the square is occupied, an 0 indicates that it is
  // free space.
  readMap(oGrid);   // Read a map in from the file map.txt
  printMap(oGrid);  // Print the map on the screen
  

 

  // Plan handling
  // 
  // A plan is an integer, n, followed by n doubles (n has to be
  // even). The first and second doubles are the initial x and y
  // (respectively) coordinates of the robot, the third and fourth
  // doubles give the first location that the robot should move to, and
  // so on. The last pair of doubles give the point at which the robot
  // should stop.
  // vector<double> newPlan;
  pLength = readPlanLength(); // Find out how long the plan is from plan.txt
  plan = new double[pLength]; // Create enough space to store the plan
  readPlan(plan, pLength);    // Read the plan from the file plan.txt.
  printPlan(plan,pLength);    // Print the plan on the screen
  writePlan(plan, pLength);   // Write the plan to the file plan-out.txt
  
  
 // Proportional Control variables
  int x = 2;
  int y = 3;

  double startX = -6;
  double startY = -6;
  bool propCtrl = true;

  
// Wavefront Algorithm
// Coordinate to map: (x+8)*2, Map to coordinate: (x/2)-8


int wave = 2;
int wavecheck = 2;
bool finished = false;
  int i = 29;
  int j = 29;

 double startingCellX = 4;
 double startingCellY = 4;
  int arr[] = {1, 0, -1, 0, 0, 1, 0, -1};
 
oGrid[j][i] = wave;
 while (!finished)
   {
      std::cout << "Current Wave: " << wave << std::endl;
      
      //check every cell
      //for (int y = 0; y < SIZE; y++) 
     for (int y = SIZE - 1; y >= 0; y--)
      {
         //for (int x = 0; x < SIZE; x++)
	 for (int x = 0; x < SIZE; x++)
         {
             printf("X:%2d, Y:%2d\n", x, y);
            if (oGrid[y][x] == wave) //oGrid[y][x] == wave
            {
               std::cout << "current node: " << oGrid[y][x] << std::endl;
               int move = 0;
               for (int i = 0; i < 4; i++)
               {
                  // {1, 0, -1, 0, 0, 1, 0, -1}
                  int finalX = x + arr[move]; 
                  int finalY = y + arr[move+1];
 
                  int skipX = x + arr[move] + arr[move];
                  int skipY = y + arr[move+1] + arr[move+1]; 
                  move+=2;
                  
                  //std::cout << "move " << move << std::endl;
                  
                  if ((finalX < 0) || (finalY < 0) || (finalX >= SIZE) || (finalY >= SIZE))
                     continue;
		  //Check if neighboring cells are free/0
                  if (oGrid[finalY][finalX] == 0 && oGrid[finalY][finalX] != 1 /*&& oGrid[skipY][skipX] != 1*/)
                  {  
                     oGrid[finalY][finalX] = wave + 1; //Increase by 1
                    
                     // Check if start goal is reached
                     if ((finalX == startingCellX) && (finalY == startingCellY))
                     {
                        // target backwards to the start goal
                        finished = true;
                        //std::cout << "Wave Number on Node: " << oGrid[finalY][finalX] << std::endl; 
                       // std::cout << "Reached start!!! " << std::endl;
                        
                     }
                  }

               } // neighbor check
            } // if
         } // x values of grid
      } // y values of grid
     
     wave++;
}
//End of Wavefront

printMap(oGrid);

  //Start of Pathfinding
  vector<double> newPlan; // Vector that contains path (includes trailpoints and waypoints)
  int newStrtX = startingCellX;
  int newStrtY = startingCellY;
  for (int waveStep = wave-1; waveStep >= 2; waveStep--)
   {
      
      // search for neigboring cells
      
      int move = 0;
      for (int i = 0; i < 4; i++)
      {
           
         // compute final position
        int nx = newStrtX + arr[move];
        int ny = newStrtY + arr[move+1];
        move+=2;
   
         if ((nx < 0) || (ny < 0) || (nx >= SIZE) || (ny >= SIZE))
            continue;

         // Check if cell is in current wave step
        
         if (oGrid[ny][nx] == waveStep)
         {
            newStrtX = nx;
            newStrtY = ny;
            //std::cout << "Building Path!!" << std::endl;
            //Check to ensure backtrack
            //std::cout << "You're looking at: " << oGrid[ny][nx] <<std::endl;
            //Add to vector: coordinate to map conversion 
            newPlan.push_back((nx)/2-8);
            newPlan.push_back((ny)/2-8);
             
         }
      } // for (all neighbors)
   
   } // for (all wave steps to go back)*/
 // End of path finding  
 newPlan.push_back(6.5);
 newPlan.push_back(6.5);
 
 
 //std::cout << "Now deleting coordinates " <<std::endl;
 

//(-6 -6), -5 -6, -5 -6, -4 -6, -4 -6, -3 -6, -3 -6, -2 -6, (-2 -6), -2 -5, -2 -5, -2 -4, -2 -4, -2 -3, -2 -3, -2 -3, -2 -2, -2 -2, -2 -1, -2 -1, -2 0, -2 0, -2 1, -2 1, (-2 2), -1 2, -1 2, 0 2, 0 2, 1 2, 1 2, 2 2, 2 2, 2 2, (2 3), 3 3, 3 3, 3 3, 4 3, 4 3, 5 3, 5 3, 6 3, (6 3), 6 4, 6 4, 6 5, 6 5, 6 6, (6 6), (6.5 6.5) added 

//If both the x and y changes, this signifies a change in direction so add coordinate that's before change of x/y in a vector (plan)

vector<double> modPlan;
modPlan.push_back(newPlan[0]);
modPlan.push_back(newPlan[1]);
double pointx = newPlan[0];
double pointy = newPlan[1];
int iter = 0;
 for (int i=0; i<newPlan.size(); i++) {
    //std::cout << "pointX " << pointx <<std::endl;
    //std::cout << "pointY " << pointy <<std::endl;
    //std::cout << "newPlan[x] " << newPlan[iter] <<std::endl;
    //std::cout << "newPlan[y] " << newPlan[iter+1] <<std::endl;
    if ((pointx != newPlan[iter]) && (pointy != newPlan[iter+1])) {
       
       //std::cout << "No Match!! "<<std::endl;
       modPlan.push_back(newPlan[iter-2]);
       modPlan.push_back(newPlan[iter-1]);
       
       pointx = newPlan[iter-2];
       pointy = newPlan[iter-1];
       if (pointx == newPlan[newPlan.size()-1] && pointy == newPlan[newPlan.size()-2]) {
          break;
       }
    }
    iter+=2;
 }

// modified file of waypoints


ofstream modFile("modPlan-out.txt");
  for (int i = 0; i < modPlan.size(); i++) {
     
      modFile << modPlan[i] << " ";
  }
  
  modFile.close();

//Print to newPlan.txt file
 ofstream MyFile("newPlan.txt");
  for (int i = 0; i < newPlan.size(); i++) {
     
      MyFile << newPlan[i] << " ";
  }
  
  MyFile.close();
 
  readMap(nGrid);
  writeMap(nGrid, modPlan);  // Write a map out to the file map-out.txt
  readReverseMap(nGrid);
  writeReverseMap(nGrid);

// Main control loop
while(true) 
    {   
      // Update information from the robot.
      robot.Read();
      // Read new information about position
      pose = readPosition(lp);
      // Print data on the robot to the terminal
      printRobotData(bp, pose);
      // Print information about the laser. Check the counter first to stop
      // problems on startup
      if(counter > 2){
	printLaserData(sp);
      }

      // Print data on the robot to the terminal --- turned off for now.
      // printRobotData(bp, pose);
      
      // If either bumper is pressed, stop. Otherwise just go forwards
     std::cout << std::endl;
     std::cout << "Prop Ctrl: " << propCtrl << std::endl;
     //double theta = ; //Theta
     if(bp[0] || bp[1]){
	speed= 0;
	turnrate= 0;
	propCtrl = false;
      } else if(propCtrl == true){
	turnrate = (atan2((modPlan[y] - startY), (modPlan[x] - startX))) - pose.pa; //Proportional Control for turnrate
	if (turnrate > 0 && turnrate <0.001 || turnrate < 0 && turnrate > -0.0001) { //Because 0 is never reached
		turnrate = 0;
		speed = (1.0/(sqrt(pow(modPlan[x] - startX,2) + pow(modPlan[y] - startY,2)))) * sqrt(pow(modPlan[x] - pose.px,2) + pow(modPlan[y] - pose.py,2)); // Proportional Control for speed
		if (speed > 0 && speed <0.05) {  //Because 0 is never reached
		  speed = 0;
		  startX = pose.px;    //set new starting point
		  startY = pose.py;
		  x += 2;              // For array transversal
                  y += 2;
		  if (y >= modPlan.size()) {        //At end of plan
			speed = 0;
			turnrate = 0;	
			break;
                  } 
               }
        }
      } 
      if(!bp[0] && bp[1] && propCtrl == false) {
	speed = -1;
	turnrate = 1.5;
      } 
      if(bp[0] && !bp[1] && propCtrl == false) {
	speed = -1;
	turnrate = -1.5;
      }
      if(bp[0] && bp[1] && propCtrl == false) {
	speed = -1;
	turnrate = 1.5;
      }
      if(!bp[0] && !bp[1] && propCtrl == false) {
	speed = .3; 
	turnrate = 0;
	if(sp.MinLeft() > 1.3 && sp.MinRight() > 1.3) {
		speed = 0;
		turnrate = 0;
		startX = pose.px;    //set new starting point
		 startY = pose.py;
		propCtrl = true;}
      }
      // Goal: Find a way to make the starting x and starting y to not be the goal. Take whatever current pose and snapshot that pose into the starting x and y.
      // Once the speed reaches 0 or just finishes navigation to a certain point, the numbers within plan change.
      
      
      // What are we doing?
      std::cout << "Speed: " << speed << std::endl;      
      std::cout << "Turn rate: " << turnrate << std::endl << std::endl;
      std::cout << " X Target " << modPlan[x] << std::endl << std::endl; 
      std::cout << " Y Target " << modPlan[y] << std::endl << std::endl;

      std::cout << " Starting X " << startX;  
      std::cout << " Starting Y " << startY; 



     
      //std::cout << " theta " << theta << std::endl;	 
      // Send the commands to the robot
      pp.SetSpeed(speed, turnrate);  
      // Count how many times we do this
      counter++;
    } //end of main loop 
} // end of main() 





  
  


/**
 * readMap
 *
 * Reads in the contents of the file map.txt into the array map
 * in such a way that the first element of the last row of the
 * file map.txt is in element [0][0].
 *
 * This means that whatever is in the file looks like the occupancy
 * grid would if you drew it on paper.
 *
 **/


void readMap(int map[SIZE][SIZE])
{
  std::ifstream mapFile;
  mapFile.open("map.txt");


  for(int i = SIZE - 1; i >= 0; i--){
    for(int j = 0; j < SIZE; j++)
      {
	mapFile >> map[i][j];
      }
  }
  
  mapFile.close();

} // End of readMap()

/**
 * printMap
 *
 * Print map[][] out on the screen. The first element to be printed
 * is [SIZE][0] so that the result looks the same as the contents of
 * map.txt
 *
 **/

void printMap(int map[SIZE][SIZE])
{
  for(int i = SIZE -1; i >= 0; i--){
    for(int j = 0; j < SIZE; j++)
      {
	printf("%2d ", map[i][j]);
      }
    std::cout << std::endl;
  }
} // End of printMap()

/**
 * writeMap
 *
 * Write a map into map-out.txt in such a way that the [0][0] element
 * ends up in the bottom left corner of the file (so that the contents
 * of the file look like the relevant occupancy grid.
 *
 **/

void writeMap(int map[SIZE][SIZE], vector<double> modPlan)
{
  std::ofstream mapFile;
  mapFile.open("revmap-out.txt");
  //(X + 8)*2 modPlan[0] = -6, modPlan[1] = -6, (-6+8)*2 = 4, @ map[4][4] = 2
  
  //reverse(modPlan.begin(), modPlan.end());
   int lx = 0;
   for(int i = 0; i < SIZE -1; i++) {  //for(int i = SIZE - 1; i >= 0; i--){    
   
    for(int j = 0; j < SIZE; j++)
      {
            if (  i == (modPlan[lx+1]+8)*2 && j == (modPlan[lx]+8)*2) { //  (modPlan[ly]+8/2) && (modPlan[lx]+8/2)
		mapFile << 2 << " ";
               
                //std::cout << "lx: " << lx << endl;
                //std::cout <<  "Coord -> map ModPlanX at this point: "<< (modPlan[lx]+8)*2 << endl;
                //std::cout <<  "Map ModPlanX at this point: "<< modPlan[lx] << endl;
                //std::cout <<  "Coord -> map ModPlanY at this point: "<< (modPlan[lx+1]+8)*2 << endl;
                //std::cout <<  "Map ModPlanY at this point: "<< modPlan[lx+1] << endl;
                lx+=2;
                
               
          
            }  else {
            mapFile << map[i][j] << " ";
            //printf("I:%2d, J:%2d\n", i, j);  
          }    
      }
 
    mapFile << std::endl;
  }

  mapFile.close();
}



void readReverseMap(int map[SIZE][SIZE])
{
  std::ifstream mapFile;
  mapFile.open("revmap-out.txt");


  for(int i = SIZE - 1; i >= 0; i--){
    for(int j = 0; j < SIZE; j++)
      {
	mapFile >> map[i][j];
      }
  }
  
  mapFile.close();

} 

void writeReverseMap(int map[SIZE][SIZE])
{
  std::ofstream mapFile;
  mapFile.open("map-out.txt");
  
  
  
   for(int i = 0; i < SIZE -1; i++) {    
   
    for(int j = 0; j < SIZE; j++)
      {
            mapFile << map[i][j] << " ";    
      }
 
    mapFile << std::endl;
  }

  mapFile.close();
}

        
/**
 * readPosition()
 *
 * Read the position of the robot from the localization proxy. 
 *
 * The localization proxy gives us a hypothesis, and from that we extract
 * the mean, which is a pose. 
 *
 **/

player_pose2d_t readPosition(LocalizeProxy& lp)
{

  player_localize_hypoth_t hypothesis;
  player_pose2d_t          pose;
  uint32_t                 hCount;

  // Need some messing around to avoid a crash when the proxy is
  // starting up.

  hCount = lp.GetHypothCount();

  if(hCount > 0){
    hypothesis = lp.GetHypoth(0);
    pose       = hypothesis.mean;
  }

  return pose;
} // End of readPosition()


void printLaserData(LaserProxy& sp)
{

  double maxRange, minLeft, minRight, range, bearing;
  int points;

  maxRange  = sp.GetMaxRange();
  minLeft   = sp.MinLeft();
  minRight  = sp.MinRight();
  range     = sp.GetRange(5);
  bearing   = sp.GetBearing(5);
  points    = sp.GetCount();

  //Uncomment this to print out useful laser data
  //std::cout << "Laser says..." << std::endl;
  //std::cout << "Maximum distance I can see: " << maxRange << std::endl;
  //std::cout << "Number of readings I return: " << points << std::endl;
  //std::cout << "Closest thing on left: " << minLeft << std::endl;
  //std::cout << "Closest thing on right: " << minRight << std::endl;
  //std::cout << "Range of a single point: " << range << std::endl;
  //std::cout << "Bearing of a single point: " << bearing << std::endl;

  return;
} // End of printLaserData()

/**
 *  printRobotData
 *
 * Print out data on the state of the bumpers and the current location
 * of the robot.
 *
 **/

void printRobotData(BumperProxy& bp, player_pose2d_t pose)
{

  // Print out what the bumpers tell us:
  std::cout << "Left  bumper: " << bp[0] << std::endl;
  std::cout << "Right bumper: " << bp[1] << std::endl;
  // Can also print the bumpers with:
  //std::cout << bp << std::endl;

  // Print out where we are
  std::cout << "We are at" << std::endl;
  std::cout << "X: " << pose.px << std::endl;
  std::cout << "Y: " << pose.py << std::endl;
  std::cout << "A: " << pose.pa << std::endl;

  
} // End of printRobotData()

/**
 * readPlanLength
 *
 * Open the file plan.txt and read the first element, which should be
 * an even integer, and return it.
 *
 **/

int readPlanLength(void)
{
  int length;

  std::ifstream planFile;
  planFile.open("plan.txt");

  planFile >> length;
  planFile.close();

  // Some minimal error checking
  if((length % 2) != 0){
    std::cout << "The plan has mismatched x and y coordinates" << std::endl;
    exit(1);
  }

  return length;

} // End of readPlanLength

/**
 * readPlan
 *
 * Given the number of coordinates, read them in from plan.txt and put
 * them in the array plan.
 *
 **/

void readPlan(double *plan, int length)
{
  int skip;

  std::ifstream planFile;
  planFile.open("plan.txt");

  planFile >> skip;
  for(int i = 0; i < length; i++){
    planFile >> plan[i];
  }

  planFile.close();

}

 // End of readPlan

/**
 * printPlan
 *
 * Print the plan on the screen, two coordinates to a line, x then y
 * with a header to remind us which is which.
 *
 **/

void printPlan(double *plan , int length)
{
  std::cout << std::endl;
  std::cout << "   x     y" << std::endl;
  for(int i = 0; i < length; i++){
    std::cout.width(5);
    std::cout << plan[i] << " ";
    if((i > 0) && ((i % 2) != 0)){
      std::cout << std::endl;
    }
  }
  std::cout << std::endl;

} // End of printPlan


/**
 * writePlan
 * 
 * Send the plan to the file plan-out.txt, preceeded by the information
 * about how long it is.
 *
 **/


void writePlan(double *plan, int length)
{
 
  
  std::ofstream planFile;
  planFile.open("plan-out.txt");

  planFile << length << " ";
  for(int i = 0; i < length; i++){
    planFile << plan[i] << " ";
  }
  
  planFile.close();
 
} // End of writePlan
