#include "vex.h"
#include "odometry.h"
#include "pid.h"
#include "globals.h"

class line_intersection {

  float pathx[100], pathy[100];
  float path_resolution = 20;
  int last_found_index = 0;
  float goal_x = 0, goal_y = 0;


float QuadBezier(float p0, float p1, float p2,
                 float t) { // interpolates between the 3 given points
  float pFinal = 0, cp = 0;
  // Quadratic Bezier Formula
  cp = p1 * 2 - (p0 + p2) / 2;
  pFinal = pow((1 - t), 2) * p0 + (1 - t) * 2 * t * cp + t * t * p2;
  return pFinal;
}


// calculate the curve points and save them as an array

void bezier_path_x(float p0, float p1, float p2, int fidelity) {
  float t = 0;
  for (int i = 1; i <= fidelity + 1; i++) {
    t = (float)i / fidelity;
    if (t < 1) {
      pathx[i - 1] = QuadBezier(p0, p1, p2, t);
    }
  }
}

// calculate the curve points and save them as an array
void bezier_path_y(float p0, float p1, float p2, int fidelity) {
  float t = 0;
  for (int i = 1; i <= fidelity + 1; i++) {
    t = (float)i / fidelity;
    if (t < 1) {
      pathy[i - 1] = QuadBezier(p0, p1, p2, t);
    }
  }
}





  void goal_pt_search(float look_ahead_dist) {
    float x1, y1, x2, y2, dx, dy, dr, D, discriminant;
    float sol_x1, sol_y1, sol_x2, sol_y2, sol_pt1_x, sol_pt1_y, sol_pt2_x, sol_pt2_y, min_x, min_y, max_x, max_y;

    bool intersect_found = false;
    int starting_index = last_found_index;

    for (int i = starting_index; starting_index < i < (path_resolution - 1); i++) {
      //beginning of line-circle intersection code
        x1 = pathx[i] - chassis.x;
        y1 = pathy[i] - chassis.y;
        x2 = pathx[i+1] - chassis.x;
        y2 = pathy[i+1] - chassis.y;
        dx = x2 - x1;
        dy = y2 - y1;
        dr = sqrt(dx*dx + dy*dy);
        D = x1*y2 - x2*y1;
        discriminant = (look_ahead_dist*look_ahead_dist) * (dr*dr) - D*D;

         if (discriminant >= 0) {
            sol_x1 = (D * dy + signum(dy) * dx * sqrt(discriminant)) / dr*dr;
            sol_x2 = (D * dy - signum(dy) * dx * sqrt(discriminant)) / dr*dr;
            sol_y1 = (- D * dx + fabs(dy) * sqrt(discriminant)) / dr*dr;
            sol_y2 = (- D * dx - fabs(dy) * sqrt(discriminant)) / dr*dr;
            
            sol_pt1_x  =sol_x1 + chassis.x;
            sol_pt1_y  =sol_y1 + chassis.y;
            sol_pt2_x  =sol_x2 + chassis.x;
            sol_pt2_y  =sol_y2 + chassis.y;
            // # end of line-circle intersection code
            
            min_x = min(pathx[i], pathx[i+1]);
            min_y = min(pathy[i], pathy[i+1]);
            max_x = max(pathx[i], pathx[i+1]);
            max_x = max(pathy[i], pathy[i+1]);
         

             //# if one or both of the solutions are in range
            if (((min_x <= sol_pt1_x <= max_x) and (min_y <= sol_pt1_y <= max_y)) or ((min_x <= sol_pt2_x <= max_x) and (max_y <= sol_pt2_y <= max_y))) {
            
                intersect_found = true;
            
                //# if both solutions are in range, check which one is better
                if (((min_x <= sol_pt1_x <= max_x) and (min_y <= sol_pt1_y <= max_y)) and ((min_x <= sol_pt2_x <= max_x) and (max_y <= sol_pt2_y <= max_y))) {
                    //# make the decision by comparing the distance between the intersections and the next point in path
                    if (chassis.length(sol_pt1_x, sol_pt1_y, pathx[i+1], pathy[i+1]) < chassis.length(sol_pt2_x, sol_pt2_y, pathx[i+1], pathy[i+1])) {
                        goal_x = sol_pt1_x;
                        goal_y = sol_pt1_y;
                   } else {
                        goal_x = sol_pt2_x;
                        goal_y = sol_pt2_y;
                   }
               // # if not both solutions are in range, take the one that's in range
               } else {
                   // # if solution pt1 is in range, set that as goal point
                    if ((min_x <= sol_pt1_x <= max_x) and (min_y <= sol_pt1_y <= max_y)) {
                        goal_x = sol_pt1_x;
                        goal_y = sol_pt1_y;
                  } else {
                        goal_x = sol_pt2_x;
                        goal_y = sol_pt2_y;
                    }
                }

                   //# only exit loop if the solution pt found is closer to the next pt in path than the current pos
                if ((chassis.length(goal_x, goal_y, pathx[i+1], pathy[i+1])) < (chassis.length(chassis.x, chassis.y, pathx[i+1], pathy[i+1]))) {
              //# update lastFoundIndex and exit
                    last_found_index = i;
                    break;
                }
      
                else {
                   // # in case for some reason the robot cannot find intersection in the next path segment, but we also don't want it to go backward
                    last_found_index = i+1;
                }     

                    
            //# if no solutions are in range
             } else {
                    intersect_found = false;
                    //# no new intersection found, potentially deviated from the path
                    //# follow path[lastFoundIndex]
                    int test =last_found_index;
                    goal_x = pathx[test];
                    goal_y = pathy[last_found_index];

            }
        }
    }
  }
};

extern line_intersection get_line;