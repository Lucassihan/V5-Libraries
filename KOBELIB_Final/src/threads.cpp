#include "vex.h"
#include "movement.h"
#include <iostream>
#include "auton.h"
#include "driver.h"

int autonMotors() {
  while (true) {
  if (move.autondriveenabled) {
  move.movementloop();
  }
  delay(10); 
  }
}

int collectData() {
  chassis.resetData();
  while (true) {
  chassis.collectdata();
  chassis.updateOdom();
  delay(4);
  }
  return 0;
}


void disableRobot() {
  move.autondriveenabled = false;
  velocitydrive(0, 0);
  drivestopping(0);
}



bool terminal_print = true;
bool brain_screen_print = false;




 int dataSend() {

//   //int count = 8;
//   while (true) {


    
//     if (terminal_print) {
//     delay(0);
//     /*
// if (Brain.timer(msec) > 2300) {
//     if (count > 2) {
//       count = 0;
//     printf("\n");
//     printf("(%.2f, %.2f)",(Brain.timer(msec) - startime)/10,  2 + (fl.power() + ml.power() + bl.power()) / 3);
//     printf(",(%.2f, %.2f)",(Brain.timer(msec) - startime)/10,  (fr.power() + mr.power() + br.power()) / 3);

//     } else {
//     printf(",(%.2f, %.2f)",(Brain.timer(msec) - startime)/10,  2 + (fl.power() + ml.power() + bl.power()) / 3);
//     printf(",(%.2f, %.2f)",(Brain.timer(msec) - startime)/10,  (fr.power() + mr.power() + br.power()) / 3);    
//     }
//     count ++;
// }
// */

//  // printf("Cata: %7.3f ", cata.temperature(celsius));
// //  printf("torque %7.3f,", cata.torque());

  
//   printf("tx %7.3f,", chassis.perpindiculartracker *( 2.75 * M_PI /360));
//   printf("  ty %7.3f,", chassis.paralleltracker *( 2.75 * M_PI /360));
//   //printf("  slew %7.3f ", drivePID.slewAmount);
//   //rintf("  E %7.3f ", drivePID.error);
//   //printf("  E %7.3f ", move.straighterror);
//   //printf("  stg %7.3f ", move.straightlinegoal);
// /*
//   printf("CV %7.3f,", cataPID.output);
// */


//   printf("C %7.3f,", cataRotation.angle());

//   printf("tx %7.3f,", chassis.perpindiculartracker *( 2.75 * M_PI /360));
//   printf("  ty %7.3f,", chassis.paralleltracker *( 2.75 * M_PI /360));


//   printf("  H %7.3f ", chassis.h);
  
//   printf("  X %7.3f", chassis.x);
//   printf("  Y %7.3f ", chassis.y);


//   //printf("  XT %5.2f ", chassis.xtarg);
//  //printf("  YT %5.2f ", chassis.ytarg);
//   //printf("  PA %5.2f ", chassis.pointangle(chassis.xtarg, chassis.ytarg));

//  // printf("  Pdist %5.2f ", chassis.pointdist());
  
//   printf("  time %5.2f ", (Brain.timer(msec) - starttime)/1000);
//   printf(" endtime %5.3f ", (endtime) / 1000);
  
//   //printf(" Youtput %5.3f ", drivePID.output);
//   //printf(" Houtput %5.3f ", turnPID.output);
//   //printf(" Rpm %5.3f ", TY.velocity(rpm));
//   //printf(" E %5.3f ", turnPID.sumerror);

//     printf("\n");

//     }

//     if (brain_screen_print) {
//     Brain.Screen.setFillColor(black);
//     Brain.Screen.setFont(propXL);
//     Brain.Screen.setCursor(1, 1);
//     Brain.Screen.clearLine();
//     Brain.Screen.print("ytrack: %3.2f", chassis.paralleltracker*( 2.75 * M_PI /360));
//     Brain.Screen.setCursor(2, 1);
//     Brain.Screen.clearLine();
//     Brain.Screen.print("xtrack: %3.2f", chassis.perpindiculartracker*( 2.75 * M_PI /360));
//     Brain.Screen.setCursor(3, 1);
//     Brain.Screen.clearLine();
//     Brain.Screen.print("X: %3.2f", chassis.x);
//     Brain.Screen.setCursor(4, 1);
//     Brain.Screen.clearLine();
//     Brain.Screen.print("Y: %3.2f", chassis.y);
//     Brain.Screen.setCursor(5, 1);
//     Brain.Screen.clearLine();
//     if (cata.temperature(celsius) > 44.0) Brain.Screen.setFillColor(orange);
//     Brain.Screen.print("H: %3.2f  C:%3.2f V:%3.2f D:%1.1d", chassis.h, cata.temperature(celsius), cata.voltage(), move.down_enough);
//     Brain.Screen.setCursor(6, 1);
//     Brain.Screen.clearLine();
//     if (boosted_shots) {
//     Brain.Screen.setFillColor(red);
//     Brain.Screen.print("boosted");
//     } else {
//        Brain.Screen.setFillColor(green);
//     Brain.Screen.print("weak");   
//      }

  
//      std::cout << "" << std::flush;
//      Controller.Screen.clearLine(1);
//      Controller.Screen.setCursor(1, 1);
//      Controller.Screen.print(" C:%2.0f, V:%2.0f", cata.temperature(celsius), cata.voltage());
//     delay(50);

//    }
//   return 0;
  
// }
// }

//TURN BACK ON ******************************************
// Brain.Screen.setFillColor(white);
// Brain.Screen.setFont(monoL);
// Brain.Screen.setPenColor(black);

while(1){
//Print
/*
Brain.Screen.setCursor(1, 1);
Brain.Screen.print("Kobe");
Brain.Screen.setCursor(1, 8);
Brain.Screen.print(CataRotation.angle(degrees));
delay(100);
Brain.Screen.clearLine();

*/
// Brain.Screen.setCursor(2, 1);
// Brain.Screen.clearLine();
// Brain.Screen.print("X %3.3f", X());

// Brain.Screen.setCursor(3, 1);
// Brain.Screen.clearLine();
// Brain.Screen.print("Y %3.3f", Y());

// Brain.Screen.setCursor(4, 1);
// Brain.Screen.clearLine();
// Brain.Screen.print("H %3.3f", IMU());

// KEY STUFF
                    // Brain.Screen.setCursor(2, 1);
                    // Brain.Screen.clearLine();
                    // Brain.Screen.print("  H %7.1f ", chassis.x);

                    // Brain.Screen.setCursor(3, 1);
                    // Brain.Screen.clearLine();
                    // Brain.Screen.print("  H %7.1f ", chassis.y);

                    // Brain.Screen.setCursor(4, 1);
                    // Brain.Screen.clearLine();
                    // Brain.Screen.print("  H %7.1f ", chassis.h);


/*
Brain.Screen.setCursor(1, 1);
Brain.Screen.clearLine();
Brain.Screen.print("testl %d FBar %f", testl.pressing(), flAng());
*/

// offset finder

//KOBE HAD THIS ON -----------------------------------------------------------------------
    // Brain.Screen.setCursor(1, 1);
    // Brain.Screen.clearLine();
    // Brain.Screen.print("IMU %5.2f 1 %5.2f 2 %5.2f", chassis.h, IMU1.rotation());
    // Brain.Screen.setCursor(2, 1);
    // Brain.Screen.clearLine();
    // Brain.Screen.print("ytrack: %3.2f", chassis.paralleltracker*( 2 * M_PI /360));
    // Brain.Screen.setCursor(3, 1);
    // Brain.Screen.clearLine();
    // Brain.Screen.print("xtrack: %3.2f", chassis.perpindiculartracker*( 2 * M_PI /360));
//-----------------------------------------------------------------------------------------


          //changed for 2 inch tracking wheels
// imu mutliplier finder
//


//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11
// Brain.Screen.setCursor(2, 1);
// Brain.Screen.clearLine(); 
// Brain.Screen.setFont(monoXL);
// Brain.Screen.print("Kobe Sam");

/*
Brain.Screen.setCursor(3, 1);
Brain.Screen.clearLine();
Brain.Screen.print("htype %d rt %4.2f %d", htype, degToIn(rtrack.position(deg), 2.75), rtrack.position(deg));
*/



//Brain.Screen.print("%1.1f%1.1f%1.1f%1.1f%1.1f%1.1f%1.1f%1.1f", YCurve[5],XCurve[5],YCurve[10],XCurve[10],YCurve[15],XCurve[15],YCurve[20],XCurve[20]);
//Brain.Screen.print("X%1.1f Y%1.1f X%1.1f Y%1.1f", QuadBezier(0, 5, 30, 0.375), QuadBezier(0, 15, 30, 0.375), QuadBezier(0, 5, 30, 0.75), QuadBezier(0, 5, 30, 0.75));

//Brain.Screen.print("L %2.2f R %2.2f", frontL.position(deg), frontLR.position(deg));


// temp disable

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11
// Brain.Screen.setCursor(3, 1);
// Brain.Screen.clearLine();
//Brain.Screen.print("IMU %5.2f 1 %5.2f 2 %5.2f", chassis.h, IMU1.rotation() / 0.9872222222, IMU2.rotation() / 0.9902777778);

//KOBE HAD THIS ON ------------------------------------------
// Brain.Screen.setCursor(4, 1);
// Brain.Screen.clearLine();
// Brain.Screen.print("X %3.3f Y %3.3f", chassis.x, chassis.y);
//-------------------------------------------------------------

// TURN BACK ON
printf("%7.1f", chassis.x);
printf(", %7.1f ", chassis.y);
printf("  H %7.1f ", chassis.h);

// printf(" 1 %7.1f", IMU1.rotation());
// printf(", 2 %7.1f ", IMU2.rotation());



  delay(1000);
  // was delay 200 but too quick
std::cout<<std::flush;
}
return 0;
}