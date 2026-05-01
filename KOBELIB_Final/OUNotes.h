/////////////////////////////////////////////////////////////////////

// void WorldSkills(){
//   //delay(5000);
//   auto_settings();
//   TKD = 6.548;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);
  
//   TKP = TKP * 1.15;

//   move.cataEnabled = false;

//   leftWing.set(1);

//   //b curve -8.728, -18.495 and -24.251, -21.154
//   //move.BCurveTo(-8.7, -10.5, -24.3, -14.9, 12, 12, YKP, TKP, 12, 8, 5);
//   // change to 2 b tos
//   move.bto(-8.7, -10.5, 12, 12, YKP, TKP*1.2, 12, 5.5);
//   move.bto(-24.3, -14.9, 12, 12, YKP, TKP*1.2, 12, 5.5);
//   move.face(90, TKP, TKI, TKD, 12, 4.5);
//   move.straighth(-40, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(600);

//   // to -12.063, -16.595
//   move.facePoint(-12.1, -18.6, TKP*1.2, TKI, TKD*1.2,  12, 4, false);
//   move.to(-12.1, -18.6, 12, 12, YKP, TKP, 12, 3.5);
//   leftWing.set(0);

//   // face -19.239
//   // aim was 339
//   move.face(339, TKP*1.2, TKI, TKD*1.2, 12, 4); 
//   // was 337.5

//   //move.straighth(-4, 331, YKP, YKD, TKP, TKD, 12, 12, 12, 0.5);  

//   // // b to  -9.599, -17.202 
//   // move.bto(-9.599, -17.202, 12, 12, YKP, TKP, 12, 3.5);
//   // move.face(-19.239, TKP, TKI, TKD, 12, 0);
//    rightWing.set(1);
//    move.face(339, TKP*1.2, TKI, TKD*1.2, 12, 0); 
//    // was 337.5

//   /////MATCHLOAD///////////////////
//   cata.setVelocity(68, percent);
//   intake.setVelocity(68, percent);
//   cata.spin(reverse);
//   intake.spin(reverse);
//   delay(23000); // wait for 23 seconds to shoot
//   rightWing.set(0);

//   //delay(500);

//   ////////////////////////////////

//   cata.stop();
//   intake.stop();


//   // add descore by face descore down then b curve -7.571, -13.333 and 0.760,  -7.463
//   move.facePoint(-7.6, -13.3, TKP*1.2, TKI, TKD*1.2,  12, 4, true); 
//   leftWing.set(1); 
//   rightWing.set(1);
//   move.BCurveTo(-7.6, -13.3, 1.1,  -7.5, 12, 12, YKP, TKP, 12, 12.5, 5);
//   move.swingOnRight(180, TKP, TKI, TKD, 12, 0);
//   delay(600);
//   leftWing.set(0);

//   // b curve 4.0,   5.2 and 6.6,  31.7
//   move.BCurveTo(6.0, 5.2, 6.6, 31.7, 12, 12, YKP, TKP, 12, 18, 9);
//   // x1 was 4.0

//   // b curve 5.6,  77.7 and 4.8,  85.6
//   move.BCurveTo(6.6,  77.7, 3.8,  88.6, 12, 12, YKP*0.8, TKP, 12, 17, 5);
//   // x1 was 5.6 but changed to 6.6
//   // bl was 8

//   // face then wing down then b curve 1.3,  91.1 and -6.3, 100.1 then wing up
//   move.facePoint(1.3,  91.1, TKP*1.2, TKI, TKD*1.2,  12, 8, true); 
//   leftWing.set(1);
//   move.BCurveTo(1.3,  91.1, -6.3, 100.1, 12, 12, YKP, TKP, 12, 12, 5);
//   move.swingOnRight(90, TKP, TKI, TKD, 12, 0);
//   delay(600);
//   leftWing.set(0);

//   move.voldrive(3, 3);
//   delay(700);

//   // b to -18.0, 104.4    
//   move.facePoint(-18.0, 104.4, TKP*1.2, TKI, TKD*1.2,  12, 4, true);
//   rightWing.set(1);  
//   move.bto(-18.0, 104.4, 12, 12, YKP, TKP, 12, 9);
//   //delay(800);

//   // move.swingOnRight(100, TKP, TKI, TKD, 12, 0);  
//   // // was 90
//   // delay(300);

//   // swing then ram at 90
//   move.straighth(-80, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(400);
//   move.straighth(9.5, 90, YKP*1.3, YKD, TKP, TKD, 12, 12, 12, 5.5);
//   move.straighth(-80, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(400);
//   move.straighth(8, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 5.5);
//   rightWing.set(0);

//   // b face to -15.4, 88.0
//   move.facePoint(-15.4, 88.0, TKP*1.2, TKI, TKD*1.2,  12, 4, true);  
//   leftWing.set(1);
//   rightWing.set(1);
//   move.bto(-15.4, 80.0, 12, 12, YKP, TKP, 12, 5.5);
//   // b to -22.7, 77.7
//   move.bto(-40.7, 71.7, 8, 8, YKP, TKP*1.3, 12, 9);
//   // bl was 5.5
//   // swing on left into 180 net
//   //move.swingOnLeft(110, TKP, TKI, TKD, 8, 14);
//   move.face(110, TKP*1.2, TKI, TKD*1.2, 12, 7); 
//   rightWing.set(0);
//   move.swingOnLeft(180, TKP, TKI, TKD, 8, 10);
//   move.straighth(-30, 180, YKP, YKD*1.2, TKP, TKD, 12, 12, 12, 0);
//   delay(600);
//   // rightWing.set(0);
//   // leftWing.set(0);
//   delayRelease();

//   // b to with high turn kp -23.7, 61.0
//   move.bto(-23.7, 61.0, 12, 12, YKP, TKP*1.35, 12, 11);
//   leftWing.set(1);
//   rightWing.set(1);

//   // b to -48.7, 58.6 (also high turn kp) 
//   move.facePoint(-52.7, 58.6, TKP*1.2, TKI, TKD*1.2,  12, 4, true);
//   move.bto(-52.7, 58.6, 12, 12, YKP, TKP*1.15, 12, 8);

//   // swing on left into net at 180
//   move.face(180, TKP*1.2, TKI, TKD*1.2, 12, 6);
//   rightWing.set(0);
//   move.straighth(-60, 180, YKP, YKD, TKP, TKD, 7, 12, 12, 0);
//   delay(900);
//   delayRelease();

//   //  b to -50.6, 68.7 high turn kp
//   move.bto(-50.6, 68.7, 12, 12, YKP, TKP*1.3, 12, 5.5);
//   leftWing.set(1);

//   //  b to with higher bl  -65.2, 54.2 
//   move.bto(-65.2, 54.2, 12, 12, YKP, TKP*1.05, 12, 5.5); 
//   move.face(180, TKP*1.2, TKI, TKD*1.2, 12, 7);
//   //rightWing.set(1);
//   move.straighth(-50, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(900);
//   //leftWing.set(0);
//   delayRelease();

//   // to -62.1, 56.2 
//   move.bto(-62.1, 51.2, 12, 12, YKP, TKP*1.35, 12, 7); 

//   // b to with high turn kp -84.5, 49.3
//   move.face(90, TKP*1.3, TKI, TKD*1.3, 12, 0);
//   delay(200);
//   rightWing.set(1);
//   move.bto(-88.5, 51.8, 12, 12, YKP, TKP*1.35, 12, 7); 

//  // swing on left into angled push
//  move.face(180, TKP, TKI, TKD*1.3, 12, 14);
//  move.swingOnLeft(230, TKP, TKI, TKD, 12, 14);
//   leftWing.set(1);
//   move.straighth(-30, 230, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(400);
//   move.face(180, TKP, TKI, TKD*1.3, 12, 8);
//   move.straighth(-40, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 7, 12, 12, 0);
//   delay(500);
//   move.straighth(7, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 7, 12, 12, 5);
//   //move.straighth(17, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 9);
//   // was 20 units
//   rightWing.set(0);
//   leftWing.set(0);

//   // b to with high turn kp -111.5, 73.2
//   move.facePoint(-109.5, 73.2, TKP*1.2, TKI, TKD*1.2,  12, 4, true);
//   move.bto(-111.5, 73.2, 12, 12, YKP, TKP*1.15, 12, 7);
//   move.swingOnLeft(180, TKP, TKI, TKD, 12, 14);
//   leftWing.set(1);

//   // b  -111.8, 88.0 
//   move.bto(-111.8, 88.0, 12, 12, YKP, TKP*1.15, 12, 7);
//   rightWing.set(1);

//   // descore b -103.8, 99.9
//   move.bto(-103.8, 99.9, 12, 12, YKP, TKP*1.15, 12, 7);

//   // swing on left to verify the descore
//   move.swingOnLeft(270, TKP, TKI, TKD, 12, 0);  
//   delay(400);
//   rightWing.set(0);

//   // vol drive up and out
//   move.voldrive(3, 3);
//   delay(400);

//   // b to -94.3, 102.7
//   move.bto(-94.3, 102.7, 12, 12, YKP, TKP*1.15, 12, 7);

//   // swing against wall and face 270 
//   move.swingOnLeft(270, TKP, TKI, TKD, 12, 0);

//   // pure ram code
//   move.straighth(-30, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(400);
//   move.straighth(9.5, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 5.5);
//   move.straighth(-30, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(400);
//   move.straighth(8, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 5.5);
//   leftWing.set(0);

//   // go hang
//   // face to -113.8, 82.6
//   move.facePoint(-120.1, 82.6, TKP*1.2, TKI, TKD*1.2,  12, 8, false);  
//   move.to(-120.1, 82.6, 12, 12, YKP, TKP*1.2, 12, 9);
//   BarHang.set(1);
//   move.swingOnLeft(168, TKP, TKI, TKD, 12, 9);
//   // hang macro
//   move.straighth(100, 176, YKP*1.4, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(1400);
//   // move.voldrive(-3, -3);
//   // delay(200);
//   BarHang.set(0);
//   PistonWheel.set(1); 


//   autonStop();

//   ////////////////////////////////////// SKILLS REDO

//   // b face then to -21.8,    56.6
//   move.facePoint(-21.8, 56.6, TKP*1.2, TKI, TKD*1.2,  12, 4, true);  
//   leftWing.set(1);
//   rightWing.set(1);
//   move.bto(-21.8, 56.6, 12, 12, YKP, TKP, 12, 5.5);
//   move.swingOnLeft(135, TKP, TKI, TKD, 12, 6);

//   // b to -44.0, 73.2
//   move.bto(-44.0, 75.5, 8, 12, YKP*0.8, TKP, 12, 5.5);
//   move.swingOnLeft(180, TKP, TKI, TKD, 12, 6);
//   move.voldrive(-7, -7);
//   delay(700);
//   move.voldrive(4, 4);
//   delay(300);
//   leftWing.set(0);
//   rightWing.set(0);

//   // b curve -46.5, 62.4 and -57.0, 49.8
//   //move.CurveTo(-51.5, 55.4, -57.0, 56.8, 12, 12, YKP, TKP, 12, 8, 5);
//   //move.facePoint(-51.5, 52.4, TKP*1.2, TKI, TKD*1.2,  12, 11, false);  
//   move.to(-51.5, 52.4, 12, 12, YKP*0.8, TKP, 12, 11);
//   move.facePoint(-57.0, 52.8, TKP*1.2, TKI, TKD*1.2,  12, 12, false);  
//   move.to(-57.0, 52.8, 12, 12, YKP*0.8, TKP, 12, 8.4);

//   // swing on left to 0
//   //move.swingOnRight(0, TKP, TKI, TKD, 12, 6);
//   move.face(0, TKP*1.2, TKI, TKD*1.2, 12, 8);

//   //leftWing.set(1);

//   // ram
//   move.straighth(60, 0, YKP, YKD, TKP, TKD, 7, 12, 12, 0);
//   delay(900);

//   move.voldrive(-4, -4);
//   delay(200);
//   leftWing.set(0);
//   rightWing.set(0);

//   // to -57.4, 59.9 
//   move.to(-57.4, 59.9, 12, 12, YKP, TKP, 12, 5.5);

//   // face back to -82.6, 58.2 
//   move.facePoint(-91.6, 56.2, TKP*1.2, TKI, TKD*1.2,  12, 4, true);  
//   rightWing.set(1);
//   move.bto(-91.6, 56.2, 12, 12, YKP, TKP, 12, 5.5);

//   // wing dynamic
//   //move.face(180, TKP, TKI, TKD, 6, 20);
//   //move.face(236, TKP, TKI, TKD, 7, 14);
//   move.face(230.264, TKP, TKI, TKD, 12, 4); 
//   leftWing.set(1);
//   move.straighth(-30, 230.264, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(500);
//   move.face(180, TKP, TKI, TKD*1.3, 12, 8);
//   move.straighth(-40, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 7, 12, 12, 0);
//   delay(700);
//   move.straighth(17, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 9);
//   // was 20 units
//   rightWing.set(0);
//   leftWing.set(0);

//   // face to -110.7, 79.8
//   move.facePoint(-110.7, 79.8, TKP*1.2, TKI, TKD*1.2,  12, 4, false);  
//   //rightWing.set(1);
//   move.to(-110.7, 79.8, 12, 12, YKP, TKP, 12, 5.5);
//   //rightWing.set(0);

//   // face b to -115.4, 89.2 
//   move.facePoint(-115.4, 89.2, TKP*1.2, TKI, TKD*1.2,  12, 4, true); 
//   leftWing.set(1); 
//   move.bto(-115.4, 89.2, 12, 12, YKP, TKP, 12, 5.5);

//   // face b to -105.1, 105.9 
//   move.facePoint(-105.1, 100.1, TKP*1.2, TKI, TKD*1.2,  12, 4, true);  
//   rightWing.set(1); 
//   move.bto(-105.1, 100.1, 12, 12, YKP, TKP, 12, 0);
//   delay(700);

//   // copy paste descore and score
//   move.swingOnLeft(270, TKP, TKI, TKD, 12, 0);
//   delay(600);
//   rightWing.set(0);

//   move.voldrive(3, 3);
//   delay(600);

//   leftWing.set(1);

//   // b to -94.5, 107.2
//   move.bto(-94.5, 104.1, 12, 12, YKP, TKP, 12, 5.5);

//   // swing turn into ram
//   move.swingOnLeft(270, TKP, TKI, TKD, 12, 0);  
//   delay(700);

//   // swing then ram at 90
//   move.straighth(-30, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(400);
//   move.straighth(9.5, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 5.5);
//   move.straighth(-30, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(400);
//   move.straighth(8, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 5.5);
//   leftWing.set(0);

//   // go hang
//   // face to -109.1,    83.8
//   move.facePoint(-113.1, 83.8, TKP*1.2, TKI, TKD*1.2,  12, 4, false);
//   BarHang.set(1);
//   move.to(-113.1, 83.8, 12, 12, YKP, TKP, 12, 4);

//   // left swing to face 180
//   move.swingOnLeft(180, TKP, TKI, TKD, 12, 5.5);

//   // to -112.6, 38.0
//   move.to(-112.6, 34.0, 12, 12, YKP, TKP, 12, 0);
//   delay(1550);

//   BarHang.set(0);
//   delay(200);
//   PistonWheel.set(1);

//   autonStop();

//   // hang macro
//   move.straighth(100, 180, YKP*1.4, YKD, TKP, TKD, 9, 12, 12, 0);
//   delay(1450);
//   // move.voldrive(-3, -3);
//   // delay(200);
//   BarHang.set(0);
//   PistonWheel.set(1); 

// autonStop();
// }

// void WorldsNewFarSide(){
//   //delay(5000);
//   auto_settings();
//   TKD = 6.548;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);
  
//   TKP = TKP * 1.15;

//   move.cataEnabled = false;

//   // drag coord: -19.2, -14.5 
//   // old drag back was -13.6, 43.8

//   // intake to -13.7, 44.6
//   // first ball
//       intake.setVelocity(100, percent);
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);
//       deployIntake(); // thread
//       move.to(-13.7, 48.6, 12, 12, YKP, TKP, 12, 10);
//       // b to -15.6, 44.8
//       //  move.bto(-13.6, 43.8, 12, 12, YKP, TKP, 12, 0);
//       //  delay(200);
//       leftWing.set(0);

//     //move.bto(-19.2, -14.5, 12, 12, YKP, TKP, 12, 8);

//   // swing shift into net
//     //move.swingOnRight(90, TKP, TKI, TKD, 12, 18);
//     move.face(120, TKP*1.2, TKI, TKD*1.6, 12, 14);
//       intake.setVelocity(100, percent);
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
//   move.straighth(60, 90, YKP*1.4, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(700);
//   move.straighth(-10, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 5);

//   // second ball
//   // intake face to -26.4, 57.3
//   move.facePoint(-26.4, 57.3, TKP*1.2, TKI, TKD*1.2,  12, 8, false);
//       intake.setVelocity(100, percent);
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);
//   move.to(-35.4, 57.3, 12, 12, YKP, TKP*1.4, 12, 9);
//   move.bto(-17.6, 43.8, 12, 12, YKP, TKP, 12, 12);

//   // into net with second ball
//       //move.swingOnLeft(90, TKP, TKI, TKD, 12, 15);
//       move.face(90, TKP*1.2, TKI, TKD*1.2, 12, 15);
//       intake.setVelocity(100, percent);
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
//   move.straighth(40, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(800);

//   // third ball
//   // b to -14.3, 44.2
//   move.bto(-14.3, 44.2, 12, 12, YKP, TKP, 12, 10);
//   // intake to -29.8, 35.1 
//     move.facePoint(-29.8, 33.1, TKP*1.2, TKI, TKD*1.2,  12, 15, false);
//       intake.setVelocity(100, percent);
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);
//   move.to(-29.8, 33.1, 12, 12, YKP, TKP*1.4, 12, 8);

//   // to -6.3, 16.4 with high turn kp
//   move.bto(-4.3, 16.4, 12, 12, YKP, TKP*1.32, 12, 9);

//   // to -5.0, 0.5 with high turn kp
//   move.to(-0.1, -5.5, 12, 12, YKP, TKP*1.32, 12, 8);

//   // face 90 then outtake
//   move.face(90, TKP*1.2, TKI, TKD*1.2, 12, 15);
//       intake.setVelocity(100, percent);
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
//   delay(250);

//   // face point intake to -29.2, -7.1
//   move.facePoint(-34.2, -0.1, TKP*1.2, TKI, TKD*1.2,  12, 15, false);
//       intake.setVelocity(100, percent);
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);
//   move.to(-34.7, -0.1, 12, 12, YKP, TKP*1.4, 12, 8);

//   // b to 5.4, 1.7 then wing down
//   move.bto(5.4, 1.7, 12, 12, YKP, TKP*1.4, 12, 8);
//   leftWing.set(1);
//   rightWing.set(1);
//   // b to 17.7, 11.1 then wing up
//   move.bto(17.7, 11.1, 12, 12, YKP, TKP*1.4, 12, 8);

//   // swing on right to face 180
//   move.swingOnRight(180, TKP*1.2, TKI, TKD, 12, 0); 
//   delay(300);
//   leftWing.set(0);

//   move.face(200, TKP*1.2, TKI, TKD*1.2, 12, 15);
//   move.straighth(-70, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(600);
//   move.straighth(15, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(220);
//   rightWing.set(0);

//   move.face(25, TKP*1.2, TKI, TKD*1.2, 12, 15);
//       intake.setVelocity(100, percent);
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
//   move.straighth(70, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(600);
//   move.straighth(-10, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 6);




//   /////////////////////////////////REDO FROM HERE
//   autonStop();
//   ////////////////////////////////////////////////
//   // into net
//     move.bto(-13.6, 43.8, 12, 12, YKP, TKP*1.4, 12, 7);
//       move.face(90, TKP*1.2, TKI, TKD*1.2, 12, 15);
//       intake.setVelocity(100, percent);
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
//   move.straighth(40, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(700);

//   // alliance ball
//   // b to -16.7, 48.7 
//   move.bto(-16.7, 48.7, 12, 12, YKP, TKP, 12, 7);
//   // intake to 5.9, 10.0
//   move.facePoint(-2, 10.0, TKP*1.2, TKI, TKD*1.2,  12, 15, true);
//       intake.setVelocity(100, percent);
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);
//   move.bto(-2, 10.0, 12, 12, YKP, TKP*1.4, 12, 8);

//   // gate ball 
//   // face to -7.7, -3.1 
//   move.facePoint(-7.7, -3.1, TKP*1.2, TKI, TKD*1.2,  12, 15, false);
//       intake.setVelocity(100, percent);
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);
//   move.to(-7.7, -3.1, 12, 12, YKP, TKP*1.4, 12, 6);
//   // face intake to -30.3, -5.6 
//   move.facePoint(-34.3, -1.6, TKP*1.2, TKI, TKD*1.2,  12, 15, false);
//   move.to(-34.3, -1.6, 12, 12, YKP, TKP*1.4, 12, 6);

//   // b to 3.5, -3.8
//   move.bto(3.5, -3.8, 12, 12, YKP, TKP*1.4, 12, 8);
//   // b face to 17.4, 6.2 wit descore wing
//   move.facePoint(17.4, 6.2, TKP*1.2, TKI, TKD*1.2,  12, 15, true);
//   leftWing.set(1);
//   move.bto(17.4, 6.2, 12, 12, YKP*1.3, TKP*1.4, 12, 6);
//   move.swingOnRight(180, TKP*1.2, TKI, TKD, 12, 0);
//   delay(400);
//   leftWing.set(0);

//   // face to 18.6, 12.7
//   move.facePoint(18.6, 15.7, TKP*1.2, TKI, TKD*1.2,  12, 15, false);
//   move.to(18.6, 15.7, 12, 12, YKP*1.3, TKP*1.4, 12, 0);
//   delay(400);
//   // pure ram outtake 
//       move.face(0, TKP*1.2, TKI, TKD*1.2, 12, 15);
//       intake.setVelocity(100, percent);
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
//   move.straighth(40, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(300);
//   move.straighth(-12, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 8);
//   move.straighth(40, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(500);
//   move.straighth(-12, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 6);



//   autonStop();

//   // intake to -25.7, 37.9
//   move.facePoint(-25.7, 37.9, TKP*1.2, TKI, TKD*1.2,  12, 15, true);
//       intake.setVelocity(100, percent);
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);
//   move.to(-25.7, 37.9, 12, 12, YKP, TKP, 12, 8);

//   // into net
//     move.bto(-13.7, 48.6, 12, 12, YKP, TKP, 12, 10);
//     move.face(90, TKP*1.2, TKI, TKD, 12, 15);
//       intake.setVelocity(100, percent);
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
//   move.straighth(60, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(700);
//   move.straighth(-10, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 5);

// autonStop();
// }

// void WorldsFarSide(){
//   delay(5000);
//   auto_settings();
//   TKD = 6.548;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);
  
//   TKP = TKP * 1.15;

//   move.cataEnabled = false;

//   deployIntake();

//   //b curve 4.5, -9.7 and 14.5, -19.0
//   move.BCurveTo(4.5, -9.7, 16.5, -16.0, 7, 7, YKP, TKP, 12, 8, 9);
//   // x2 was 16.5
//   move.face(270, TKP, TKI, TKD, 12, 0);
//   delay(300);
//   move.straighth(-40, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(400);
//   // // forward
//   move.straighth(5, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(100);

//   // // face to -2.6, 8.3
//   // move.facePoint(-2.6, 8.3, TKP*1.2, TKI, TKD*1.2,  12, 15, false);
//       intake.setVelocity(100, percent);
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);
//   // move.to(-2.6, 8.3, 12, 12, YKP, TKP, 12, 6);

//   // // face intake to -3.8, 31.5 
//   // move.facePoint(-3.8, 31.5, TKP*1.2, TKI, TKD*1.2,  12, 15, false);
//   // move.to(-3.8, 31.5, 12, 12, YKP*1.2, TKP, 12, 2); 

//   move.CurveTo(-1.6, 8.3, -2.8, 31.5, 12, 12, YKP*1.4, TKP*1.2, 12, 7, 5);

//   // b to -0.9, -2.0
//   move.bto(-0.9, -2.0, 12, 12, YKP*1.2, TKP, 12, 14); 

//   // face b to then descore swing 6.1,   -13.4 
//   move.facePoint(6.1, -13.4 , TKP*1.2, TKI, TKD*1.2,  12, 15, true);
//   leftWing.set(1);
//   move.bto(6.1, -13.4 , 12, 12, YKP, TKP, 12, 5); 

// // // // kick out
// move.swingOnRight(270, TKP, TKI, TKD, 12, 0);
// delay(100);
// leftWing.set(0);

// // move.voldrive(3, 3);
// // delay(200);

// // b face to 18.0, -17.7
//   move.facePoint(18.0, -19.7, TKP*1.2, TKI, TKD*1.2,  12, 15, true);
//   move.bto(18.0, -19.7, 12, 12, YKP, TKP, 12, 8); 

// // 270 ram code
//   move.face(270, TKP, TKI, TKD, 12, 14);
//   move.straighth(-40, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(400);
//   // forward
//   move.straighth(9, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 5.5);
//   //delay(200);

//   // face to 23.6, 16.5
//   move.facePoint(23.6, 16.5, TKP*1.2, TKI, TKD*1.2,  12, 15, false);
//   move.to(23.6, 16.5, 12, 12, YKP, TKP, 12, 8); 

//   // face to 43.7, 24.0 
//   move.facePoint(46.7, 24.0, TKP*1.2, TKI, TKD*1.2,  12, 15, false);
//   move.to(46.7, 24.0, 12, 12, YKP, TKP, 12, 8); 

//   // swing face to
//   move.swingOnRight(180, TKP, TKI, TKD, 12, 9);

//   // // face b to 24.2, 11.4
//   // move.facePoint(24.2, 11.4, TKP*1.2, TKI, TKD*1.2,  12, 15, false);
//   // move.to(24.2, 11.4, 12, 12, YKP, TKP, 12, 8); 

//   // // face b to 39.2, 16.2 
//   // //move.facePoint(44.2, 16.2, TKP*1.2, TKI, TKD*1.2,  12, 15, true);
//   // rightWing.set(1);
//   // move.bto(49.2, 16.2, 12, 12, YKP, TKP, 12, 8); 

//   // // right wing swing
//   // move.face(0, TKP*1.4, TKI, TKD, 12, 0);
//   // delay(100);
//   // move.face(90, TKP*1.4, TKI, TKD, 12, 0);
//   // delay(100);
//   // move.face(180, TKP*1.2, TKI, TKD, 12, 7);
//       intake.setVelocity(100, percent);
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
//   rightWing.set(0);
//   move.straighth(40, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(600);

//   move.straighth(-15, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 4);
//   //move.face(0, TKP*1.2, TKI, TKD, 12, 7);

//   // face intake to 53.1, 31.7
//     move.facePoint(53.1, 31.7, TKP*1.2, TKI, TKD*1.2,  12, 15, false);
//       intake.setVelocity(100, percent);
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);
//   move.to(53.1, 31.7, 12, 12, YKP, TKP, 12, 6);

//   move.straighth(-10, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 4);

//   // into net
//   move.face(180, TKP*1.2, TKI, TKD*1.2, 12, 7);
//       intake.setVelocity(100, percent);
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
//   rightWing.set(0);
//   move.straighth(40, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(600);

//   // face intake to 37.9, 29.1
//   move.facePoint(37.9, 29.1, TKP*1.2, TKI, TKD*1.2,  12, 15, false);
//       intake.setVelocity(100, percent);
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);
//   move.to(37.9, 29.1, 12, 12, YKP, TKP, 12, 6);

  



// autonStop();

// // face to 21.8, 13.5 with high bl then to 42.1, 12.6  
//   move.facePoint(21.8, 9.5, TKP*1.2, TKI, TKD*1.2,  12, 14, false);
//   move.to(21.8, 9.5, 12, 12, YKP*1.2, TKP, 12, 20);
//   delay(200); // settle
//   move.facePoint(42.1, 10.6, TKP*1.2, TKI, TKD*1.2,  12, 14, false);
//   move.to(42.1, 10.6, 12, 12, YKP*1.2, TKP*1.2, 12, 5); 

// // into net
//   move.face(180, TKP, TKI, TKD, 12, 8);
//       intake.setVelocity(100, percent);
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
//   move.straighth(40, 180, YKP, YKD, TKP, TKD, 8, 12, 12, 0);
//   delay(200);

//   // b to 33.5, 18.5
//   move.bto(33.5, 20.5, 12, 12, YKP, TKP*1.2, 12, 20);
//       intake.setVelocity(100, percent);
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);

//   // intake face to 57.5, 18.3 
//   move.facePoint(57.5, 15.3, TKP*1.2, TKI, TKD*1.2,  12, 18, false);
//   move.to(57.5, 15.3, 7, 12, YKP, TKP*1.2, 12, 9);

//   // outtake to 180
//     move.face(180, TKP, TKI, TKD, 12, 12);
//       intake.setVelocity(100, percent);
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
//   delay(200);

//   // intake face to 57.4, 30.1
//   move.facePoint(57.4, 30.1, TKP*1.2, TKI, TKD*1.2,  12, 14, false);
//       intake.setVelocity(100, percent);
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);
//   move.to(57.4, 30.1, 8, 12, YKP, TKP*1.2, 12, 5);

//   // face to 52.0, 18.5
//   move.facePoint(52.0, 18.5, TKP*1.2, TKI, TKD*1.2,  12, 15, false);
//   move.to(52.0, 18.5, 12, 12, YKP, TKP*1.2, 12, 18);

//   // outtake at 180 again
//       move.face(180, TKP, TKI, TKD, 12, 14);
//       intake.setVelocity(100, percent);
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
//   delay(200);

//   // intake face to 37.4, 31.4 
//   move.facePoint(35.4, 33.4, TKP*1.2, TKI, TKD*1.2,  12, 14, false);
//       intake.setVelocity(100, percent);
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);
//   move.to(35.4, 33.4, 12, 12, YKP, TKP*1.2, 12, 8);

//   // wings down then b to 57.9, 23.1
//   leftWing.set(1);
//   rightWing.set(1);
//   move.bto(50.9, 23.1, 12, 12, YKP, TKP*1.2, 12, 11);
//   move.face(0, TKP, TKI, TKD, 12, 8);
//   move.straighth(-40, 0, YKP, YKD, TKP, TKD, 8, 12, 12, 0);
//   delay(500);

//   move.straighth(10, 0, YKP, YKD, TKP, TKD, 8, 12, 12, 6);

//   move.face(180, TKP, TKI, TKD, 12, 8);
//       intake.setVelocity(100, percent);
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse); 
//   move.straighth(50, 180, YKP, YKD, TKP, TKD, 8, 12, 12, 0);
//   delay(500);



//   autonStop();

//   // face then to -15.1, 51.6
//   move.facePoint(-15.1, 47.6, TKP*1.2, TKI, TKD*1.2,  12, 5.5, false);
//   move.to(-15.1, 47.6, 12, 12, YKP, TKP, 12, 6);
//   leftWing.set(0);

//   delay(200); // settle

//   // drag back to -4.1, -2.7 
//   move.bto(-4.7, -2.7, 12, 12, YKP, TKP*0.5, 12, 9);

//   // face 75 degrees then outtake
//   move.face(65, TKP*1.2, TKI, TKD*1.2, 12, 5.5);
//       intake.setVelocity(100, percent);
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
//   delay(300);

//       intake.setVelocity(100, percent);
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);

//   // face to -35.5, -2.3
//   move.facePoint(-28.5, -0.3, TKP*1.2, TKI, TKD*1.2,  12, 5.5, false);
//   move.to(-28.5, -0.3, 12, 12, YKP, TKP, 12, 6);

//   delay(200);

//   // b to -0.3, -0.6  
//   move.bto(-7.2, -0.6, 12, 12, YKP*0.8, TKP*1.1, 12, 8);

//   // face b to 8.8, 11.5
//   move.facePoint(13.8, 11.5, TKP*1.2, TKI, TKD*1.2,  12, 5.5, true);
//   leftWing.set(1);
//   move.bto(13.8, 11.5, 12, 12, YKP, TKP, 12, 0);
//   delay(600);
//   // move.swingOnRight(195, TKP, TKI, TKD, 12, 0);
//   // delay(600);
//   leftWing.set(0);

//   // swing ram
//   move.swingOnRight(180, TKP, TKI, TKD, 12, 6);
//   rightWing.set(1);  
//   move.straighth(-30, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(400);
//   move.straighth(11.5, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 5.5);
//   rightWing.set(0);

//   // forward
//   move.face(15, TKP*1.2, TKI, TKD*1.2, 12, 5.5);
//       intake.setVelocity(100, percent);
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
//   move.straighth(30, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(650);
//   move.straighth(-14.5, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 5.5);

//   // intake face to -31.0, 33.4 
//     move.facePoint(-31.0, 33.4, TKP*1.2, TKI, TKD*1.2,  12, 5.5, false);
//       intake.setVelocity(100, percent);
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);
//   move.to(-31.0, 33.4, 12, 12, YKP, TKP, 12, 6);

//   // b face to -11.7, 45.2 
//   // move.facePoint(-11.7, 45.2, TKP*1.2, TKI, TKD*1.2,  12, 5.5, true);
//   // move.bto(-11.7, 45.2, 12, 12, YKP, TKP, 12, 6);

//   // outtake at 90
//   move.face(70, TKP*1.2, TKI, TKD*1.2, 12, 5.5);
//       intake.setVelocity(100, percent);
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
//     delay(300);

//   // intake face to -29.4, 56.1
//   move.facePoint(-30.4, 56.1, TKP*1.2, TKI, TKD*1.2,  12, 5.5, false);
//       intake.setVelocity(100, percent);
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);
//   move.to(-30.4, 56.1, 12, 12, YKP, TKP, 12, 6);

//     move.face(90, TKP*1.2, TKI, TKD*1.2, 12, 5.5);
//       intake.setVelocity(100, percent);
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);

//   move.straighth(40, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(650);
//   move.straighth(-14.5, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 5.5);







//   autonStop();

//   // face intake to -28.5, 52.4
//   move.facePoint(-28.5, 52.4, TKP*1.2, TKI, TKD*1.2,  12, 5.5, false);
//       intake.setVelocity(100, percent);
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);
//   move.to(-28.5, 52.4, 12, 12, YKP, TKP, 12, 6);
//   delay(400);

//   move.face(90, TKP*1.2, TKI, TKD*1.2, 12, 5.5);
//       intake.setVelocity(100, percent);
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
//   move.straighth(50, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(800);
//   move.straighth(-7.5, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 5.5);


//   autonStop();

//   // face b to -24.6, 51.7
//   move.facePoint(-26.6, 53.7, TKP*1.2, TKI, TKD*1.2,  12, 5.5, false);
//   move.to(-26.6, 53.7, 12, 12, YKP, TKP, 12, 6);

//   // face 90 then outtake
//   move.face(90, TKP*1.2, TKI, TKD*1.2, 12, 5.5);
//       intake.setVelocity(100, percent);
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
  
//   // 








// autonStop();


// }

// void WorldsWP(){
//   //delay(5000);
//   auto_settings();
//   TKD = 6.548;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);
  
//   TKP = TKP * 1.15;

//   move.cataEnabled = false;

// deployIntake();
//       intake.setVelocity(100, percent);
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);
// move.to(11.9,  16.593, 12, 12, YKP, TKP, 12, 2);
// // x1 was 13.9
// move.face(90, TKP, TKI, TKD, 12, 2);
//       intake.setVelocity(100, percent);
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
// delay(300);
// intake.stop();
// cata.stop();

// // autonStop();


// // // // back to 7.842,  17.015
// move.bto(6.942,  17.015, 6, 12, YKP, TKP, 12, 2);
// leftWing.set(1);

// // // // kick out
// //move.face(45, TKP, TKI, TKD, 12, 5);
// move.straighth(-8, 45, YKP, YKD, TKP, TKD, 8, 12, 12, 3.4);
// move.swingOnRight(330, TKP, TKI, TKD, 12, 0);
// // was 270
// delay(600);
// //move.face(0, TKP, TKI, TKD, 12, 5);
// leftWing.set(0);

// // face to 18.2, -2.1
// move.facePoint(18.2, -2.1, TKP*1.2, TKI, TKD*1.2, 12, 12, false);
// move.to(18.2, -2.1, 12, 12, YKP, TKP*1.1, 12, 16);

// // face intake to 45.5, -10.7 
//       intake.setVelocity(100, percent);
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);
// move.facePoint(45.5, -10.7, TKP*1.2, TKI, TKD*1.2, 12, 12, false);
// move.to(45.5, -10.7, 12, 12, YKP*1.6, TKP*1.1, 12, 5.5);

// //autonStop();

// // // b to  28.813, -15.425 // was this!
// // move.facePoint(28.813, -13.425, TKP*1.2, TKI, TKD*1.2, 12, 4, true);
// // move.bto(28.813, -13.425, 12, 12, YKP, TKP*1.1, 12, 5.5);

// // // face 90
// // move.face(90, TKP, TKI, TKD, 12, 5);

// // intake to  45.729, -14.893 
//       intake.setVelocity(100, percent);
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);
// //move.to(46.729, -14.893, 12, 12, YKP, TKP, 12, 5.5);
// //move.straighthToPoint(16, 46.729, -14.893, YKP, YKD, TKP, TKD, 12, 12, 12, 5.4, false);
// // move.straighth(17, 90, YKP, YKD, TKP, TKD, 8, 12, 12, 3.4);
// // delay(200);


// // store in alley
// move.bto(22.471,   0.748, 12, 12, YKP, TKP, 12, 8);
// move.to(-3.668,   0.454, 12, 12, YKP, TKP, 12, 11);
// move.face(180, TKP, TKI, TKD, 12, 5);
//       intake.setVelocity(100, percent);  // disabled for elims
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
// delay(300);

// // face to 25.9, -9.4
// move.bto(25.9, -9.4, 12, 12, YKP, TKP, 12, 7);

// // face to 36.9, -24.2 (changed y coord)
// move.facePoint(36.9, -26.2, TKP*1.2, TKI, TKD*1.2, 12, 4, false);
// move.to(36.9, -26.2, 12, 12, YKP, TKP, 12, 5);

// // swing on left to 90
// //move.swingOnLeft(115, TKP, TKI, TKD, 12, 8);
// move.face(112, TKP, TKI, TKD, 12, 5);
//       intake.setVelocity(100, percent);  
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);
// move.straighth(9, 112, YKP, YKD, TKP, TKD, 8, 12, 12, 3.4);
// // was 12
// delay(300);

// // paste into alley 
// move.bto(22.471,   -1.748, 12, 12, YKP, TKP, 12, 8);
// // y was 2.748
// move.to(-4.668,   0.454, 12, 12, YKP, TKP, 12, 11);
// move.face(180, TKP, TKI, TKD, 12, 5);
//       intake.setVelocity(100, percent);  // disabled for elims
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
// delay(300);

// //move.straighth(29.5, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 1);
// // was 32.5

// // to -6.0, -27.8 
// move.to(-4.4, -29.8, 12, 12, YKP, TKP, 12, 1);

// autonStop();

// // move.straighth(-12, 180, YKP, YKD, TKP, TKD, 5, 12, 12, 1); // for elims
// // move.face(225, TKP, TKI, TKD, 12, 5); // for elims to bowl

// autonStop();
// }

// void WorldsCloseElim(){
//   //delay(5000);
//   auto_settings();
//   TKD = 6.548;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(1.4, 2.2, 90);
  
//   TKP = TKP * 1.15;

//   move.cataEnabled = false;

//   deployIntake();
//       intake.setVelocity(100, percent);
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);
// //move.facePoint(45.5, -10.7, TKP*1.2, TKI, TKD*1.2, 12, 12, false);
//   move.to(45.5, -10.7, 12, 12, YKP*1.3, TKP*1.05, 12, 5.5);

//   // into alley
//   //move.bto(22.471,   0.748, 12, 12, YKP, TKP, 12, 8);
//   move.bto(-3.668,   2.954, 12, 12, YKP, TKP, 12, 11);
// move.face(180, TKP, TKI, TKD, 12, 5);
//       intake.setVelocity(100, percent);  // disabled for elims
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
// delay(300);

// // going for second ball

// // face to 25.9, -9.4
// move.to(25.9, 1.9, 12, 12, YKP, TKP*1.2, 12, 7);
// // y was -9.4
// // change to forwards

// // face to 36.9, -24.2 (changed y coord)
// move.facePoint(38.9, -24.2, TKP*1.2, TKI, TKD*1.2, 12, 4, false);
// move.to(38.9, -24.2, 12, 12, YKP, TKP, 12, 5);

// // swing on left to 90
// //move.swingOnLeft(115, TKP, TKI, TKD, 12, 8);
// move.face(112, TKP, TKI, TKD, 12, 5);
//       intake.setVelocity(100, percent);  
//       intake.spin(forward);
//       cata.setVelocity(100, percent);
//       cata.spin(forward);
// move.straighth(8, 112, YKP, YKD, TKP, TKD, 8, 12, 12, 3.4);
// delay(200);

// // paste into alley 
// move.bto(22.471,   -0.748, 12, 12, YKP, TKP, 12, 8);
// // y was 2.748
// move.to(-4.668,   2.454, 12, 12, YKP, TKP, 12, 11);
// move.face(180, TKP, TKI, TKD, 12, 5);
//       intake.setVelocity(100, percent);  // disabled for elims
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
// delay(300);

// // DESCORE
// //move.to(13.9,  16.593, 12, 12, YKP, TKP*1.2, 12, 8);
// move.to(12.9,  18.593, 12, 12, YKP, TKP*1.2, 12, 8);
// move.facePoint(0.942,  5.0, TKP*1.2, TKI, TKD*1.2, 12, 4, true);
// leftWing.set(1);
// move.bto(0.942,  5.0, 6, 12, YKP, TKP*1.2, 12, 5.5);
// // y was 17.01
// move.straighth(-8, 45, YKP, YKD, TKP, TKD, 8, 12, 12, 3.4);
// move.swingOnRight(330, TKP, TKI, TKD, 12, 0);
// delay(600);
// leftWing.set(0);

// // last push then setup
// move.face(180, TKP, TKI, TKD, 12, 5);
//       intake.setVelocity(100, percent);  // disabled for elims
//       intake.spin(reverse);
//       cata.setVelocity(100, percent);
//       cata.spin(reverse);
// move.straighth(21, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 3.4);
// //move.to(-4.4, -29.8, 12, 12, YKP, TKP, 12, 1);
// move.straighth(-24, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 3.4);




// autonStop();
// }

// void ProvSkills(){
//   //delay(5000);
//   auto_settings();
//   TKD = 6.548;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);
  
//   TKP = TKP * 1.15;

//   move.cataEnabled = false;

//   // bcurve
//   //move.BCurveTo(12.497,  16.221, 30.312,  17.150, 12, 12, YKP, TKP, 12, 8, 5);

// // first two alliance balls
//   move.bto(-11.478, -18.275 , 12, 12, YKP, TKP, 12, 5.5);
//     move.face(90, TKP, TKI, TKD, 12, 4); 
//     move.straighth(-25, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(400);
//     move.straighth(5, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 1);

// // matchloader align
//     move.to(-15.630, -16.516, 12, 12, YKP, TKP, 12, 4);
//     move.face(338.296, TKP, TKI, TKD, 12, 0);
//     // was 339.496
//     // was 339.496 
//     // was 341.896
//     // was 340.896
//     delay(300);
//     rightBackWing.set(1);

//     //#2
//     // move.to(-19.130, -16.516, 12, 12, YKP, TKP, 12, 5.5);
//     // move.face(350.896, TKP, TKI, TKD, 12, 0); 
//     // rightBackWing.set(1);

// // matchload 22
// //delay(500);
// ////////////////////////////////////////////////////////////
//   move.shoot = false;
//   move.cataEnabled = false;
//   cata.setVelocity(74, percent);
//   intake.setVelocity(74, percent);
//   // was 70
//   // was 62.5 (flexwheel)
//   // was 65 percent (Mecha)
//   cata.spin(forward); // MAKE SURE TO RE ENABLE
//   intake.spin(reverse); // MAKE SURE TO RE ENABLE
//   //delay(44000); // single person matchload
//   //delay(20000); // NORMAL
//   //delay(22500); //RE ENABLE
//   delay(500); //DISABLE
//   // was 22.5 seconds
//   // was 11.5 seconds
//   intake.stop();
//   cata.stop();
//   move.cataEnabled = false;

//   //delay(500); // disable 
// /////////////////////////////////////////////////////////////
// rightBackWing.set(0);

// // curve -17.020,   6.308 and -20.261,  18.044 
// move.CurveTo(-17.020,   6.308, -24.261,  22.044, 12, 12, YKP*1.2, TKP*1.1, 12, 14, 9);
// move.face(270, TKP, TKI, TKD, 12, 9);

// leftWing.set(1);
// rightWing.set(1);

// move.straighth(77, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 17);
// // bl was 12
// //move.straighth(-5, 270, YKP*1.3, YKD, TKP, TKD, 12, 12, 12, 1);

// /*
// move.voldrive(8, 8);
// delay(600);
// */

// // move.face(315, TKP, TKI, TKD, 12, 9);
// // // was 295
// // move.face(225, TKP, TKI, TKD, 12, 9);
// // // was 245
// // // move.face(295, TKP, TKI, TKD, 12, 9);
// // // move.face(245, TKP, TKI, TKD, 12, 9);
// // move.face(270, TKP, TKI, TKD, 12, 9);

// move.voldrive(12, 12);
// delay(300);



// leftWing.set(0);
// rightWing.set(0);
// move.straighth(-5, 270, YKP*1.3, YKD, TKP, TKD, 12, 12, 12, 1);

// move.voldrive(12, 12);
// leftWing.set(1);
// rightWing.set(1);
// delay(400);

// move.straighth(-5, 270, YKP*1.3, YKD, TKP, TKD, 12, 12, 12, 1);
// leftWing.set(0);
// rightWing.set(0);

// // // DUPED
// // // to -95.239, -12.735
// // move.facePoint(-95.239, -12.735, TKP, TKI, TKD,  12, 8, false);
// // move.to(-95.239, -12.735, 12, 12, YKP*1.1, TKP*1.2, 12, 8);

// // // REDO WITH SHORT B WING

// // // b face then to -110.235,  -4.345
// // move.facePoint(-110.235, -4.345, TKP, TKI, TKD,  12, 4, true);
// // rightBackWing.set(1);
// // leftBackWing.set(1); // why not (extra corral)
// // move.bto(-110.235, -4.345, 12, 12, YKP*1.3, TKP, 12, 3); // NEEDS TO BE ACCURATE (LOW BL)

// // // turn to face 180
// // move.face(180, TKP, TKI, TKD, 12, 9);

// // // b face then to -111.931,  13.471 then left bw up
// // move.facePoint(-111.931,  13.471, TKP, TKI, TKD,  12, 4, true);
// // move.bto(-111.931,  13.471, 12, 12, YKP, TKP, 12, 8);
// // leftBackWing.set(0);

// // try again

// intake.setVelocity(100, percent);
// intake.spin(reverse);

// // face then curve -95.997,  16.421 and -100.907,  -4.506 
// move.facePoint(-95.997,  16.421, TKP*1.2, TKI, TKD*1.2,  12, 4, false);
// move.CurveTo(-95.997,  16.421, -100.907,  -4.506, 12, 12, YKP, TKP, 12, 16, 8);

// // b face then curve -116.826,   7.908 and -110.877,  91.904 
// move.facePoint(-118.826,   7.908, TKP*1.2, TKI, TKD*1.2,  12, 4, true);
// move.bto(-116.826,   7.908, 12, 12, YKP*1.3, TKP, 12, 2);
// // bl was 4
// move.face(180, TKP*1.2, TKI, TKD*1.2, 12, 5.5);
// //move.straighth(-32, 180, YKP*1.3, YKD, TKP, TKD, 12, 12, 12, 8);
// move.bto(-116.877,  91.904, 12, 12, YKP, TKP, 12, 9);

// // b face then b to -100.576,  99.176 
// move.facePoint(-100.576,  105.176, TKP*1.2, TKI, TKD*1.2,  12, 4, true);
// rightBackWing.set(1);
// move.bto(-100.576,  105.176, 12, 12, YKP, TKP, 12, 0);
// delay(700);
// move.face(270, TKP*1.2, TKI, TKD*1.2, 12, 5.5);

// // turn then b face



// //lookahead was 14

// // autonStop();

// // // face then to -101.585, -11.838 redo and try -103.669, -11.573 
// // move.facePoint(-103.669, -11.573, TKP, TKI, TKD,  12, 4, false);
// // move.to(-103.669, -11.573, 12, 12, YKP, TKP, 12, 4);

// // // swing on right
// // move.face(135, TKP, TKI, TKD, 12, 5.5);
// // leftBackWing.set(1);
// // rightBackWing.set(1);
// // move.straighth(-27, 135, YKP, YKD, TKP, TKD, 12, 12, 12, 12);
// // move.face(180, TKP, TKI, TKD, 12, 5.5);


// // // // face first point backwards then b curve -104.110, -11.496 and -111.979,  -2.652  
// // // move.facePoint(-104.110, -11.496, TKP, TKI, TKD,  12, 6, true);
// // // rightBackWing.set(1);
// // // leftBackWing.set(1);
// // // move.BCurveTo(-104.110, -11.496, -111.979,  -2.652, 12, 12, YKP*1.2, TKP*1.1, 12, 8, 5);
// // // move.voldrive(-5, -8);
// // // delay(300);
// // // rightBackWing.set(0);

// // // delay(100);

// // // move.voldrive(10, 10);
// // // delay(200);

// // cata.setVelocity(100, percent);
// // cata.spin(forward);

// // // leftBackWing.set(1);

// // // // // b curve -112.757,   1.819 and -112.831,  39.310 
// // // move.BCurveTo(-113.757,   1.819, -112.831,  39.310, 12, 12, YKP*1.2, TKP*1.1, 12, 18, 8);
// // // leftBackWing.set(0);

// // // redo 
// // rightBackWing.set(0);

// // // to -107.187,  -5.092
// // move.to(-107.187,  -5.092, 12, 12, YKP, TKP, 12, 4);

// // // b face then b curve -113.850,   1.685 and -115.543,  35.587   
// // move.facePoint(-113.850,   1.685, TKP, TKI, TKD,  12, 4, true);
// // move.BCurveTo(-113.850,   1.685, -115.543,  35.587 , 12, 12, YKP, TKP, 12, 16, 8);
// // leftBackWing.set(0);

// // // b to -112.188,  50.042 then wing down 
// // move.bto(-112.188,  55.042, 12, 12, YKP, TKP, 12, 9);
// // leftBackWing.set(1);

// // // b curve -109.981,  88.786 and -102.168,  99.788
// // move.BCurveTo(-111.981,  88.786, -102.168,  99.788, 12, 12, YKP*1.2, TKP*1.1, 12, 10, 8);
// // rightBackWing.set(1);

// // cata.stop(coast);

// // // vol descore
// // move.voldrive(-5.5, -8.5);
// // delay(420);
// rightBackWing.set(0);

// delay(100);

// move.voldrive(10, 10);
// delay(250);

// // b to -86.562, 109.808
// //move.face(170, TKP, TKI, TKD, 12, 8);
// leftBackWing.set(1);
// move.bto(-86.562, 113.808, 10, 12, YKP, TKP*1.15, 12, 0);
// delay(750);
// leftBackWing.set(0);

// intake.stop();
// cata.stop();

// //move.face(270, TKP, TKI, TKD, 12, 8);
// move.swingOnLeft(270, TKP, TKI, TKD, 12, 0);
// delay(500);
// leftBackWing.set(1);
// move.straighth(-30, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
// delay(400);
// move.straighth(9.5, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 5.5);
// move.straighth(-30, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
// delay(400);
// move.straighth(8, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 5.5);



// leftBackWing.set(0);

// // face to -90.114,  58.681
// move.facePoint(-88.114,  58.681, TKP, TKI, TKD,  12, 14, false);
// move.to(-88.114,  58.681, 12, 12, YKP, TKP, 12, 9);

// // face to -58.996,  51.858
// move.facePoint(-58.996,  51.858, TKP, TKI, TKD,  12, 14, false);
// move.to(-58.996,  51.858, 12, 12, YKP, TKP, 12, 9);

// // face to -54.426,  87.501 into vol drive
// move.facePoint(-54.426,  87.501, TKP, TKI, TKD,  12, 14, false);
// move.to(-54.426,  87.501, 12, 12, YKP, TKP, 12, 9);
// move.voldrive(12, 12);
// delay(500);

// // b to -54.943,  62.414
// move.bto(-54.943,  62.414, 12, 12, YKP, TKP, 12, 9);

// // b to -44.404,  55.137
// move.facePoint(-44.404,  55.137, TKP, TKI, TKD,  12, 14, true);
// move.bto(-44.404,  55.137, 12, 12, YKP, TKP, 12, 9);

// // face to -45.443,  87.825 into vol drive
// move.facePoint(-45.443,  87.825, TKP, TKI, TKD,  12, 14, false);
// leftWing.set(1);
// move.to(-45.443,  87.825, 12, 12, YKP, TKP, 12, 9);
// move.voldrive(12, 12);
// delay(500);
// leftWing.set(0);

// // b to -47.393,  61.083
// move.bto(-47.393,  61.083, 12, 12, YKP, TKP, 12, 9);

// // b face to -88.145,  55.366
// move.facePoint(-88.145,  55.366, TKP, TKI, TKD,  12, 14, true);
// move.bto(-88.145,  55.366, 12, 12, YKP, TKP, 12, 9);

// // b face to -66.042,  74.206
// move.facePoint(-66.042,  74.206, TKP, TKI, TKD,  12, 14, true);
// leftBackWing.set(1);
// rightBackWing.set(1);
// move.bto(-64.042,  74.206, 12, 12, YKP, TKP, 12, 4);

// // straight goal rush
// move.face(180, TKP, TKI, TKD, 12, 8);
// //move.voldrive(-12, -12);
// // change to straighth
//   move.straighth(-45, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(550);
//   move.straighth(17, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 9);
// //delay(500);
// // move.voldrive(12, 12);
// // delay(300);
// leftBackWing.set(0);
// rightBackWing.set(0);

// // to -65.829,  59.594
// move.to(-65.829,  62.794, 12, 12, YKP, TKP, 12, 9);

// // b face to -25.170,  53.145
// move.facePoint(-25.170,  55.145, TKP, TKI, TKD,  12, 14, true);
// leftBackWing.set(1);
// move.bto(-25.170,  55.145, 12, 12, YKP, TKP, 12, 9);

// // b face to -41.359,  69.158
// move.facePoint(-44.359,  69.158, TKP, TKI, TKD,  12, 14, true);
// rightBackWing.set(1);
// move.bto(-44.359,  69.158, 12, 12, YKP, TKP, 12, 9);

// // straight goal rush
// move.face(180, TKP, TKI, TKD, 12, 8);
// // move.voldrive(-12, -12);
// // delay(500);
// // move.voldrive(12, 12);
// // delay(300);
//   move.straighth(-45, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(550);
//   move.straighth(17, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 9);
// leftBackWing.set(0);
// rightBackWing.set(0);

// // b to -37.858,  56.750
// move.to(-37.858,  53.750, 12, 12, YKP, TKP, 12, 9);

// // b face then to -18.021,  85.154
// move.facePoint(-14.021,  85.154, TKP, TKI, TKD,  12, 8, true);
// leftBackWing.set(1);
// //rightBackWing.set(1);
// move.bto(-14.021,  85.154, 12, 12, YKP, TKP, 12, 8);

// leftBackWing.set(0);

// // face to 4.611,  71.881
// move.facePoint(4.611,  71.881, TKP, TKI, TKD,  12, 8, false);
// move.to(4.611,  71.881, 12, 12, YKP, TKP, 12, 6);

// // face b to 6.328,  81.93
// move.facePoint(8.328,  81.93, TKP, TKI, TKD,  12, 6, true);
// move.bto(8.328,  81.93, 12, 12, YKP, TKP, 12, 6);

// // face  b to -0.654,  93.754
// move.facePoint(-2.654,  95.754, TKP, TKI, TKD,  12, 8, true);
// leftBackWing.set(1);
// move.bto(-2.654,  95.754, 12, 12, YKP*1.15, TKP, 12, 0);
// delay(600);
// leftBackWing.set(0);


// // // b face then to -16.292, 103.141
// // move.facePoint(-16.292, 103.141, TKP, TKI, TKD,  12, 8, true);
// // move.bto(-16.292, 103.141, 6, 12, YKP, TKP, 12, 0);
// // delay(700);
// // leftBackWing.set(0);
// // rightBackWing.set(0);

// // // curve to -12.738,  91.958 and   5.155,  94.533
// // move.facePoint(5.155,  94.533, TKP, TKI, TKD,  12, 8, false);
// // move.to(5.155,  94.533, 12, 12, YKP, TKP, 12, 8);
// // //move.CurveTo(-12.738,  91.958, 5.155,  94.533, 12, 12, YKP, TKP, 12, 14, 8);

// // // b face then to -0.866, 102.514
// // move.facePoint(-0.866, 102.514, TKP, TKI, TKD,  12, 8, true);
// // rightBackWing.set(1);
// // move.bto(-0.866, 102.514, 6, 12, YKP, TKP, 12, 0);
// // delay(700);

// // // b face then to -18.129, 108.699
// // move.facePoint(-18.129, 108.699, TKP, TKI, TKD,  12, 8, true);
// // move.bto(-18.129, 108.699, 6, 12, YKP, TKP, 12, 0);
// // delay(700);

// // ram at 90 backwards
// move.swingOnRight(90, TKP, TKI, TKD, 12, 0);
// delay(500);
// rightBackWing.set(1);
// move.straighth(-30, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
// delay(400);
// move.straighth(9.5, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 5.5);
// move.straighth(-30, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
// delay(400);
// // extra
// // move.straighth(9.5, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 5.5);
// // move.straighth(-30, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
// // delay(400);
// move.straighth(8, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 5.5);
// rightBackWing.set(0);

// // face then to 7.175,  97.682
// move.facePoint(5.775,  75.682, TKP*1.2, TKI, TKD*1.2,  12, 5, false);
// BarHang.set(1);
// move.to(5.775,  75.682, 12, 12, YKP, TKP, 12, 7);

// // hang macro
//   move.face(180, TKP, TKI, TKD, 12, 7.5);
//   move.straighth(100, 180, YKP*1.4, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(1450);
//   BarHang.set(0);
//   PistonWheel.set(1); 

// //redo front pushes
// autonStop();

// // face point then to -82.651,  69.025
// move.facePoint(-82.651,  65.025, TKP, TKI, TKD,  12, 8, false);
// move.to(-82.651,  65.025, 12, 12, YKP, TKP, 12, 8);

// // normal face h -216.228 or 144 one wing out
// move.face(144, TKP, TKI, TKD*1.3, 12, 8);
// rightWing.set(1);

// // swing on left to face little below 90 then wing close

// move.swingOnLeft(82, TKP, TKI, TKD, 12, 12);
// rightWing.set(0);

// // to -59.114,  55.346
// move.facePoint(-59.114,  55.346, TKP, TKI, TKD,  12, 16, false);
// move.to(-59.114,  55.346, 12, 12, YKP, TKP, 12, 12);

// // to -60.813,  87.660 with boosted y no delay
// move.to(-60.813,  87.660, 12, 12, YKP, TKP, 12, 14);
// move.voldrive(12, 12);
// delay(600);

// // b to -56.411,  63.973
// move.bto(-56.411,  63.973, 12, 12, YKP, TKP, 12, 10);

// // b to -49.402,  53.158 then one wing out
// move.bto(-49.402,  53.158, 12, 12, YKP, TKP, 12, 5);
// leftWing.set(1);

// // to  with high bl -48.043,  76.739 then vol drive
// move.to(-52.043,  76.739, 12, 12, YKP, TKP, 12, 14);
// move.voldrive(12, 12);
// delay(600);
// leftWing.set(0);

// autonStop();

// // Old Pushes

// // dupe sweep
// move.facePoint(-84.474,  65.648, TKP, TKI, TKD,  12, 8, true);
// leftBackWing.set(1);
// move.bto(-82.474,  67.648, 12, 12, YKP, TKP, 12, 8);

// // swing on right to 90 then 214.181  
// move.swingOnRight(270, TKP, TKI, TKD, 12, 20);
// //move.swingOnRight(214.181 , TKP, TKI, TKD, 12, 14);
// move.face(214.181, TKP, TKI, TKD*1.3, 12, 8);
// leftBackWing.set(0);
// rightBackWing.set(1);

//   move.straighth(-35, 214.181, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(520);
//   move.face(180, TKP, TKI, TKD*1.3, 12, 8);
//   // bl was 4
//   //intake.spin(reverse);
//   move.straighth(-40, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(700);
//   move.straighth(17, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 9);

//   rightBackWing.set(0);

//   // to -61.455,  65.378 
//   move.to(-61.455,  65.378 , 12, 12, YKP, TKP, 12, 8);

//   // to -46.039,  54.451
//   move.to(-46.039,  54.451, 12, 12, YKP, TKP, 12, 8);

//   // b face 180
//   move.face(0, TKP, TKI, TKD*1.3, 12, 8);
//   leftWing.set(1);

//   // pure ram straighth
//   move.straighth(70, 0, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(900);
//   move.straighth(-17, 0, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 9);  
//   leftWing.set(0);

//   // to -53.332,  63.777 
//   move.bto(-53.332,  63.777, 12, 12, YKP, TKP, 12, 8);

//   // b face then wing down then b to -28.208,  57.359
//   move.facePoint(-28.208,  57.359, TKP, TKI, TKD,  12, 6, true);
//   leftBackWing.set(1);
//   move.bto(-22.208,  57.359, 12, 12, YKP, TKP, 12, 8);

//   // face 180
//   move.face(180, TKP, TKI, TKD, 12, 15);

//   // face 56.271 then both wings down
//   move.face(140, TKP, TKI, TKD*1.3, 12, 8);
//   rightBackWing.set(1);

//   // old ram code    
//   move.straighth(-30, 140, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(500);
//   move.face(180, TKP, TKI, TKD*1.3, 12, 8);
//   // bl was 4
//   //intake.spin(reverse);
//   move.straighth(-40, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(700);
//   move.straighth(17, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 9);

//   leftBackWing.set(0);
//   rightBackWing.set(0);


//   // face point then to  6.544,  84.694
//   move.facePoint(6.544,  84.694, TKP, TKI, TKD,  12, 6, false);
//   move.to(6.544,  84.694, 12, 12, YKP, TKP, 12, 8);
//   leftWing.set(1);

//   // face first point then curve  10.903,  91.809 and -10.696, 110.105
//   move.facePoint(10.903,  91.809, TKP, TKI, TKD,  12, 6, false);
//   move.CurveTo(10.903,  91.809, -7.696, 100.105, 12, 12, YKP*1.2, TKP*1.1, 12, 14, 0);
//   delay(1500);

//   // forward rams then backward ram  
//     move.face(270, TKP, TKI, TKD, 12, 0);
//     delay(400);
//     move.straighth(35, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(700);
//     move.straighth(-11, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 4.5);
//     move.straighth(35, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(400);
//     move.straighth(-11, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 4.5);
//     leftWing.set(0);
//     move.face(90, TKP, TKI, TKD, 12, 5.5);
//     rightBackWing.set(1);
//     move.straighth(-35, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(400);
//     move.straighth(6, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 4.5);
//   rightBackWing.set(0);

//   // go hang 9.138, 74.303 then hang macro
//   BarHang.set(1);
//   move.to(9.138, 74.303, 12, 12, YKP, TKP, 12, 12);
//   move.face(180, TKP, TKI, TKD, 12, 5.5);
//   move.straighth(100, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(1150);
//   BarHang.set(0);
//   PistonWheel.set(1); 

// autonStop();
// // to -16.299,   4.218
// // move.to(-16.299,   5.218, 12, 12, YKP, TKP, 12, 18);

// // // b curve -16.134,  17.065 //and -15.668,  33.941
// // move.facePoint(-16.134,  17.065, TKP, TKI, TKD,  12, 8, true);
// // move.bto(-16.134,  17.065, 12, 12, YKP, TKP, 12, 8);

// // // to -18.808,  16.216
// // move.to(-18.808,  16.216, 12, 12, YKP, TKP, 12, 8);

// // move.face(200, TKP, TKI, TKD, 12, 9);

// // move.voldrive(-50, -50);
// // delay(300);


// // // to -18.740,  23.445
// // move.to(-18.808,  16.216, 12, 12, YKP, TKP, 12, 12);
// // rightWing.set(1);

// // // b curve -25.161,  33.860 and -50.356,  36.388
// // move.BCurveTo(-25.161,  33.860, -50.356,  36.388, 12, 12, YKP*1.2, TKP*1.1, 12, 18, 13);

// // // b to -80.772,  36.214 
// // move.bto(-80.772,  36.214 , 12, 12, YKP*1.2, TKP*1.1, 12, 16);

// // // straigh h backwards heading 90
// // move.voldrive(-8, -8);
// // delay(1000);
// // rightWing.set(0);

// // to -21.273,  29.617
// move.to(-21.273,  29.617, 12, 12, YKP*1.2, TKP*1.1, 12, 16);

// // face first point then b curve -28.540,  32.953 and -41.593,  36.623
// move.facePoint(-28.540,  32.953, TKP, TKI, TKD,  12, 8, true);
// move.BCurveTo(-28.540,  32.953, -41.593,  36.623, 12, 12, YKP*1.2, TKP*1.1, 12, 18, 13);

// //rightBackWing.set(1);

// // voldrive max for 2 seconds
// // move.voldrive(-8, -8);
// // delay(2000);
// move.straighth(-70, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 12);
// move.straighth(5, 90, YKP*1.3, YKD, TKP, TKD, 12, 12, 12, 1);
// move.voldrive(-8, -8);
// delay(500);

// rightBackWing.set(0);

// // to -95.239, -12.735
// move.facePoint(-95.239, -12.735, TKP, TKI, TKD,  12, 8, false);
// move.to(-95.239, -12.735, 12, 12, YKP*1.1, TKP*1.2, 12, 8);

// // face first point backwards then b curve -104.110, -11.496 and -111.979,  -2.652  
// move.facePoint(-104.110, -11.496, TKP, TKI, TKD,  12, 6, true);
// rightBackWing.set(1);
// move.BCurveTo(-104.110, -11.496, -111.979,  -2.652, 12, 12, YKP*1.2, TKP*1.1, 12, 18, 9);
// rightBackWing.set(0);

// // to -102.398,  -5.176
// move.to(-102.398,  -5.176, 12, 12, YKP, TKP, 12, 6);
// leftBackWing.set(1);

// // b curve -112.757,   1.819 and -112.831,  39.310 
// move.BCurveTo(-112.757,   1.819, -112.831,  39.310, 12, 12, YKP*1.2, TKP*1.1, 12, 18, 5);
// leftBackWing.set(0);

// autonStop();

// // forward to -22.804,  18.153
// // face 270
// // forward to -91.929,  27.936 

// // // or curve:
// // move.to(-26.804,  20.653, 12, 12, YKP, TKP, 12, 18);
// // leftWing.set(1);
// // rightWing.set(1);
// // // leftBackWing.set(1);
// // // rightBackWing.set(1);
// // //move.CurveTo(-26.804,  18.153, -91.929,  27.936, 12, 12, YKP*1.2, TKP, 12, 19, 0);
// // //move.face(270, TKP, TKI, TKD, 12, 14);
// // move.to(-98.929,  26.936, 12, 12, YKP*1.2, TKP*1.2, 12, 0);
// // delay(1900);
// // //delay(200);
// // leftWing.set(0);
// // rightWing.set(0);
// // leftBackWing.set(0);
// // rightBackWing.set(0);

// // // back to -101.314,  -6.774
// // move.voldrive(-30, -30);
// // leftWing.set(1);
// // rightWing.set(1);
// // move.voldrive(45, 45);
// // leftWing.set(0);
// // rightWing.set(0);
// // move.voldrive(-12, -12);
// // move.bto(-101.314,  -6.774, 12, 12, YKP, TKP, 12, 16);
// // rightBackWing.set(1);

// // // b to -106.825,  -2.318 
// // move.bto(-115.825,  -2.318, 12, 12, YKP, TKP, 12, 8);


// // move.face(190, TKP, TKI, TKD, 12, 10);
// // rightBackWing.set(0);

// // // 1 m lower
// // cata.setVelocity(100, percent);
// // cata.spin(forward);

// // leftBackWing.set(1);

// // // face point then back to -117.238,  16.862
// // move.facePoint(-117.238,  16.862, TKP, TKI, TKD,  12, 8, true);
// // move.bto(-117.238,  16.862, 12, 12, YKP, TKP, 12, 8);

// // // face 180 then back to -116.475,  38.766 then wing up
// // move.face(180, TKP, TKI, TKD, 12, 10);
// // move.bto(-116.475,  38.766, 12, 12, YKP, TKP, 12, 8);
// // leftBackWing.set(0);

// // // back to -115.533,  51.118 then wing down 
// // move.bto(-115.533,  53.118, 12, 12, YKP, TKP, 12, 8);
// // leftBackWing.set(1);

// // // back to -114.010,  87.274
// // move.bto(-114.010,  87.274, 12, 12, YKP, TKP, 12, 6);

// // // swing on left to 235
// // move.swingOnLeft(235, TKP, TKI, TKD, 12, 8);
// // rightBackWing.set(1);

// // // B curve -107.524,  97.746 and -97.426, 104.913
// // move.BCurveTo(-107.524,  97.746, -96.426, 106.913, 12, 12, YKP, TKP, 12, 14, 5.5);
// // rightBackWing.set(0);

// // // turn then b to -88.979, 109.443
// // move.facePoint(-88.979, 109.443, TKP, TKI, TKD, 12, 10, true);
// // move.bto(-88.979, 109.443, 12, 12, YKP, TKP, 12, 8);

// // move.face(270, TKP, TKI, TKD, 12, 14);
// // move.straighth(-40, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
// // delay(500);
// // move.straighth(18, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 14);
// // move.straighth(-40, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
// // delay(700);
// // move.straighth(15, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 14);

// // leftBackWing.set(0);

// // // face 0
// // move.face(0, TKP, TKI, TKD, 12, 14);

// // // left back wing down then back to -88.474,  71.648 
// // leftBackWing.set(1);
// // move.bto(-84.474,  65.648, 12, 12, YKP, TKP, 12, 8);

// // // swing on right to 90 then 214.181  
// // move.swingOnRight(270, TKP, TKI, TKD, 12, 20);
// // move.swingOnRight(214.181 , TKP, TKI, TKD, 12, 14);
// // leftBackWing.set(0);
// // rightBackWing.set(1);

// //   move.straighth(-30, 214.181, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
// //   delay(500);
// //   move.face(180, TKP, TKI, TKD*1.3, 12, 8);
// //   // bl was 4
// //   //intake.spin(reverse);
// //   move.straighth(-40, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
// //   delay(700);
// //   move.straighth(17, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 9);

// // rightBackWing.set(0);







// autonStop();
// }

// void FarSideElims(){
//   //delay(5000);
//   auto_settings();
//   TKD = 6.548;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);
  
//   TKP = TKP * 1.15;

//   move.cataEnabled = false;

//   // face to -14.731,  50.124
//   rightWing.set(1);
//   move.facePoint(-14.131,  50.124, TKP*1.15, TKI, TKD, 12, 5, false);
//   rightWing.set(0);
//   intake.setVelocity(100, percent);
//   intake.spin(forward);
//   move.to(-14.131,  50.124, 12, 12, YKP*1.2, TKP*1.1, 12, 7);

//   delay(150); // settle

//   // b to  -1.022,  -2.143 
//   move.bto(-1.022,  0.543, 12, 12, YKP, TKP, 9, 5.5);
//   intake.spin(reverse); 
//   move.face(70, TKP, TKI, TKD, 12, 7);
//   delay(100);
//   //outtake
//   move.face(270, TKP, TKI, TKD, 12, 7);
//   intake.spin(forward);

//   // -33.691,  -0.586 to intake
//   move.to(-33.691,  -0.586, 12, 12, YKP, TKP, 12, 7);
//   delay(150); // settle

//   // b to -0.034,  -5.336 
//   move.bto(-2.434,  -5.336, 12, 12, YKP, TKP, 12, 9);

//   leftBackWing.set(1);

//   // b face to 10.092,   5.609 
//   move.facePoint(10.092,   5.609, TKP, TKI, TKD, 12, 5, true);
//   rightBackWing.set(1);
//   move.bto(10.092,   5.609, 12, 12, YKP*1.2, TKP, 12, 0);
//   delay(700);

//   leftBackWing.set(0);

//   intake.spin(reverse);

//   move.swingOnRight(180, TKP, TKI, TKD, 12, 0);
//   delay(500);
//   move.straighth(-40, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(600);
//   move.straighth(9.5, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 5.5);
//   rightBackWing.set(0);


//   // face to -32.718,  31.900 
//   move.facePoint(-32.718,  31.900, TKP, TKI, TKD, 12, 5, false);
//   intake.spin(forward);
//   move.to(-32.718,  31.900, 12, 12, YKP*1.2, TKP, 12, 7);

//   delay(150); 

//   // b to -18.670,  48.717
//   move.bto(-18.670,  48.717, 12, 12, YKP*1.2, TKP, 12, 7);

//   // face net drive
//   intake.spin(reverse);
//   move.face(90, TKP, TKI, TKD, 12, 7);
//   delay(100);
//   leftWing.set(1);
//   //move.voldrive(12, 12);
//   //outtake
//   //delay(500);
//   leftWing.set(0);   

//   // b to -10.602,  44.936
//   move.bto(-10.602,  44.936, 12, 12, YKP, TKP, 12, 7);

//   // face to -31.966,  55.032
//     move.facePoint(-31.966,  56.032, TKP, TKI, TKD, 12, 5, false);
//     intake.spin(forward);
//   move.to(-31.966,  56.032, 12, 12, YKP*1.2, TKP, 12, 7);

//   delay(150);

//   // b to -15.447,  55.825
//   move.bto(-15.447,  55.825, 12, 12, YKP, TKP, 12, 7);

//   // face net drive   
//   move.face(90, TKP, TKI, TKD*1.2, 12, 7);
//   intake.spin(reverse);
//   rightWing.set(1);
//   leftWing.set(1);
//   move.voldrive(12, 12);
//   delay(500);
//   leftWing.set(0);  
//   rightWing.set(0); 

//   move.bto(-15.447,  55.825, 12, 12, YKP, TKP, 12, 7);







// autonStop();
// }

// void FarSidePower(){
//   //delay(5000);
//   auto_settings();
//   TKD = 6.548;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);
  
//   TKP = TKP * 1.15;

//   move.cataEnabled = false;

//   intake.setVelocity(100, percent);

//   rightWing.set(1);
//   delay(300);
//   rightWing.set(0);

//   // forward to -7.540,  21.041
//   move.to(-7.540,  21.041, 12, 12, YKP, TKP, 12, 5.5);

//   // face point then intake to -30.353,  48.803
//   intake.spin(forward);
//   move.facePoint(-30.353,  48.803, TKP, TKI, TKD, 12, 5, false);
//   move.to(-30.353,  48.803, 12, 12, YKP, TKP, 12, 5.5);

//   // face 90
//   move.face(90, TKP, TKI, TKD, 12, 7);

//   // left wing out while rushing goal with boosted x 3.992,  44.083
//   leftWing.set(1);
//   intake.spin(reverse);
//   move.to(10.992,  44.083, 12, 12, YKP, TKP, 12, 0);
//   delay(1200);
//   leftWing.set(0);

//   // back to -8.503,  44.618
//   move.bto(-8.503,  44.618, 12, 12, YKP, TKP, 12, 5.5);

//   // face point then intake to -30.183,  35.687
//   intake.spin(forward);
//   move.facePoint(-32.183,  35.687, TKP, TKI, TKD, 12, 5, false);
//   move.to(-32.183,  35.687, 12, 12, YKP, TKP, 12, 5.5);

//   // back to -14.378,  47.736
//   move.bto(-14.378,  47.736, 12, 12, YKP, TKP, 12, 5.5);

//   // face 90
//   move.face(90, TKP, TKI, TKD, 12, 7);

//   // ram code to 4.546,  47.260
//   intake.spin(reverse);
//   move.to(12.546,  47.260, 12, 12, YKP, TKP, 12, 0);
//   delay(1100);

//   // back to -20.705,  46.346
//   move.bto(-20.705,  46.346, 12, 12, YKP, TKP, 12, 5.5);

//   // intake to 2.248,   5.467
//     intake.spin(forward);
//   move.facePoint(-3.248,   1.467, TKP, TKI, TKD, 12, 5, false);
//   move.to(-3.248,   1.467, 12, 12, YKP, TKP, 12, 5.5);

//   // face point then to 7.497,   5.708
//   // move.facePoint(9.497,   8.708, TKP, TKI, TKD, 12, 5, false);
//   // move.to(9.497,   8.708, 12, 12, YKP, TKP, 12, 8);

//   // back right wing down then face slowly to tween 0 and 90 then outtake
//   // rightBackWing.set(1);
//   // move.face(28, TKP, TKI, TKD, 12, 7);
//   // // intake.spin(reverse);
//   // // delay(400);
//   // // intake.spin(forward);
//   // move.face(270, TKP, TKI, TKD, 12, 7);
//   // rightBackWing.set(0);

//   // face point then to -12.712,  -2.979
//   move.facePoint(-12.712,  -2.979, TKP, TKI, TKD, 12, 5, false);
//   move.to(-12.712,  -2.979, 12, 12, YKP, TKP, 12, 5.5);
//   intake.spin(forward);

//   // intake to -33.517,  -2.477
//   move.facePoint(-35.517,  -2.477, TKP, TKI, TKD, 12, 5, false);
//   move.to(-35.517,  -2.477, 12, 12, YKP, TKP, 12, 5.5);

//   // back to  1.501,  -1.017
//   move.bto(1.501,  -1.017, 12, 12, YKP, TKP, 12, 5.5);

//   // back to 17.903,  13.892
//   leftBackWing.set(1);
//   move.bto(15.903,  20.392, 12, 12, YKP, TKP, 12, 9);
//   leftBackWing.set(0);

//   // pure ram side goal backwards then forwards  
//   //move.face(270, TKP, TKI, TKD, 12, 8); 
//   move.face(0, TKP, TKI, TKD, 12, 5.5);
//   //move.swingOnLeft(0, TKP, TKI, TKD, 12, 5.5);
//   leftWing.set(1);
//     move.straighth(25, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(400);
//     move.straighth(-14, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 5);
//   leftWing.set(0);
//   move.face(270, TKP, TKI, TKD, 12, 12);
//   move.face(0, TKP, TKI, TKD, 12, 12);
//       move.straighth(25, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(800);
//     move.straighth(-10, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 4.5);





//   autonStop();
// }

// void ShooterMacky(){
//     //delay(5000);
//   auto_settings();
//   TKD = 6.548;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   TKP = TKP * 1.15;

//   move.face(0, TKP, TKI, TKD, 12, 0);
//   rightBackWing.set(1);

//   cata.setVelocity(100, percent);
//   cata.spin(forward);

// move.autondriveenabled = false;
// autonStop();
// }

// void CloseSideWPAuto(){
//   //delay(5000);
//   auto_settings();
//   TKD = 6.548;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   //chassis.setPos( -0.429,   0.040, 270);
//   chassis.setPos(0, 0, 0);
  
//   TKP = TKP * 1.15;

//   move.cataEnabled = false;


// // back to  -1.537,  -8.920
//   move.facePoint( -0.994,  -5.831, TKP, TKI, TKD, 12, 5, true);
//   move.bto( -0.994,  -5.831, 3, 12, YKP, TKP, 12, 3.5);

//   leftBackWing.set(1);
//   delay(200);
//   move.face(270, TKP, TKI, TKD, 12, 8);
//   move.face(180, TKP, TKI, TKD, 12, 8);
//   move.to(0, 0, 12, 12, YKP, TKP, 12, 5.5);
//   delay(200);
//   leftBackWing.set(0);

// // new AUTO

// // // face to -48.875,  17.422
// //   intake.setVelocity(100, percent);
// //   intake.spin(forward);
// //   move.facePoint(-48.875,  17.422, TKP, TKI, TKD, 12, 5, false);
// //   move.to(-48.875,  17.422, 12, 12, YKP*1.2, TKP, 12, 7);

// //   delay(150); // settle

// // // b to 0.270,  -1.066
// //   move.bto(0.270,  -1.066, 12, 12, YKP, TKP, 12, 6);

// //   intake.spin(reverse);

// // // face outtake
// //   move.face(30, TKP, TKI, TKD, 12, 8);

// // // face to -34.175,  12.785
// //   move.facePoint(-34.175,  12.785, TKP, TKI, TKD, 12, 5, false);
// //   intake.spin(forward);
// //   move.to(-34.175,  12.785, 12, 12, YKP*1.2, TKP, 12, 7);  

// // // face to -50.990,  33.156 with intake
// //   move.facePoint(-50.990,  33.156, TKP, TKI, TKD, 12, 5, false);
// //   move.to(-50.990,  33.156, 12, 12, YKP*1.2, TKP, 12, 7);

// //   delay(150); // settle

// // // b to -4.966,  -6.462
// //   move.bto(-4.966,  -6.462, 12, 12, YKP, TKP*1.15, 12, 7);

// // // face to 2.896,   7.242 
// //   move.facePoint(2.896,   7.242, TKP, TKI, TKD, 12, 5, false);
// //   move.to(2.896,   7.242, 12, 12, YKP*1.1, TKP, 12, 7);

// //   move.face(0, TKP, TKI, TKD, 12, 4);
// //   delay(200);
// //   //move.to(6.226,  35.717, 12, 12, YKP, TKP, 12, 14);
// //   move.straighth(20, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 3);

// //   intake.spin(reverse);

// //   //outtake


// // intake ON

// // // THIS GOES FOR BALL WITH INTAKE
// // // forward to -48.285,  17.103
//  //move.face(270, TKP, TKI, TKD, 12, 8);
//  //move.to(-48.285,  17.103, 12, 12, YKP, TKP, 12, 5.5);
// // //outta
// // // forward to -42.398,  34.067 with boosted y and no bl
// //   // move.to(-42.398,  39.067, 12, 12, YKP, TKP, 12, 0);
// //   // delay(700);

// // THIS GOES TO SWEEP 2 OVER
// move.face(270, TKP, TKI, TKD, 12, 12);
// // bl was 8
// // forward to -43.600,   9.549
// leftWing.set(1);
// move.facePoint(-38.600,   9.549, TKP, TKI, TKD, 12, 5.5, false);
// leftWing.set(0);
// move.to(-38.600,   9.549, 12, 12, YKP, TKP, 12, 12);
// leftWing.set(1);
// rightWing.set(1);

// // forward to -42.962,  35.120 boosted y no bl
// move.face(0, TKP, TKI, TKD*1.4, 12, 12);
// // bl was 8
// move.to(-38.962,  40.120, 12, 12, YKP, TKP, 12, 0);
// delay(700);

// move.veldrive(0,0);
// delay(200);


// leftWing.set(0);
// rightWing.set(0);

// // // back to -41.877,  30.411
// //  move.bto(-41.877, 30.411, 12, 12, YKP, TKP, 12, 5.5);
// // // forward to -48.930,   1.030 
// // move.face(165, TKP, TKI, TKD*1.4, 12, 8);
// // intake.setVelocity(100, percent);
// // intake.spin(reverse);
// // delay(300);
// // move.face(0, TKP, TKI, TKD*1.4, 12, 8);
// //  move.bto(-48.930, -4.030, 12, 12, YKP, TKP, 12, 0);
// //  delay(700);
// //  intake.setVelocity(100, percent);
// //  intake.spin(reverse);
// //  delay(800);

// //  // back to -48.589,  30.271
//    move.bto(-40.589,  25.271, 12, 12, YKP, TKP, 12, 5.5);

//   //  move.face(270, TKP, TKI, TKD, 12, 4);

//   //  leftWing.set(1);
//   //  rightWing.set(1);

//   //  autonStop();


// //  // forward to  1.725,   8.174
//    move.facePoint(-1.872,  -3.330, TKP, TKI, TKD, 12, 5, false);
//    move.to(-1.772,  -3.330, 8, 12, YKP*0.8, TKP, 12, 5);
//    delay(200);

//   //  leftBackWing.set(1);
//   //  delay(500);
//   //  move.face(270, TKP*1.4, TKI, TKD, 12, 5.5);
//   //  move.face(180, TKP*1.4, TKI, TKD, 12, 4);
//   //  leftBackWing.set(0);

  
//   // face then to  5.015,  14.022
//     move.facePoint(5.015,  14.022, TKP, TKI, TKD, 12, 5, false);
//    move.to(5.015,  14.022, 8, 12, YKP, TKP, 12, 5);

//   // touch bar
//   //forward to 6.226,  35.717
//   //    move.facePoint(6.725,   0.174, TKP, TKI, TKD, 12, 5, false);
//   // move.to(2.725,   0.174, 12, 12, YKP, TKP, 12, 0);
//   // delay(500);



//   //click
//   move.face(0, TKP, TKI, TKD, 12, 4);
//   delay(200);
//   //move.to(6.226,  35.717, 12, 12, YKP, TKP, 12, 14);
//   move.straighth(18.5, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 3);
//   // was 35

//   intake.setVelocity(100, percent);
//   intake.spin(reverse);
//   delay(300);

//   autonStop();

//   move.straighth(-8, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 3);

//   //move.straighth(-35, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 3);
//   //autonStop();


//   // b to -4.583,  -6.873
//   move.bto(-4.583,  -8.873, 8, 12, YKP, TKP, 12, 5);

//   //move.facePoint(5.173,  -0.603, TKP, TKI, TKD, 12, 5.5, false);
//   move.face(50, TKP, TKI, TKD, 12, 4);


// autonStop();


// // outtake




//   autonStop();

// }

// void TESTSkills(){
//   delay(5000);
//   auto_settings();
//   TKD = 6.548;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);
  
//   TKP = TKP * 1.15;

//   move.cataEnabled = false;

//   // bcurve
//   //move.BCurveTo(12.497,  16.221, 30.312,  17.150, 12, 12, YKP, TKP, 12, 8, 5);

// // first two alliance balls
//   move.bto(-11.478, -18.275 , 12, 12, YKP, TKP, 12, 5.5);
//     move.face(90, TKP, TKI, TKD, 12, 4); 
//     move.straighth(-25, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(400);
//     move.straighth(5, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 1);

// // matchloader align
//     move.to(-15.630, -16.516, 12, 12, YKP, TKP, 12, 5.5);
//     move.face(339.496, TKP, TKI, TKD, 12, 0); 
//     // was 341.896
//     // was 340.896
//     rightBackWing.set(1);

// // matchload 22
// //delay(500);
// ////////////////////////////////////////////////////////////
//   move.shoot = false;
//   move.cataEnabled = false;
//   cata.setVelocity(65, percent);
//   intake.setVelocity(65, percent);
//   // was 65 percent
//   cata.spin(forward);
//   intake.spin(reverse);
//   delay(22250);
//   // was 22.5 seconds
//   // was 11.5 seconds
//   intake.stop();
//   cata.stop();
//   move.cataEnabled = false;
// /////////////////////////////////////////////////////////////
// delay(1000); //remove
// rightBackWing.set(0);


// // forward to -26.232,  19.609
// move.to(-26.232,  19.609, 12, 12, YKP, TKP, 12, 5.5);

// // wings out
// leftWing.set(1);
// rightWing.set(1);

// // face 270
// move.face(270, TKP, TKI, TKD, 12, 5);

// // forward to -93.488,  28.210
// move.to(-100.488,  28.210, 12, 12, YKP*1.4, TKP, 12, 0);
// // was x = -93
// delay(2500);
// leftWing.set(0);
// rightWing.set(0);

// // back by 10
// move.straighth(-6, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 2.5);

// // back to loader -97.897, -13.440
// move.bto(-97.897, -16.440, 12, 12, YKP, TKP, 12, 5.5);

// // shoot code with turn
//     move.face(15.496, TKP, TKI, TKD, 12, 0); 
//     // was 341.896
//     // was 340.896
//     leftBackWing.set(1);

// // matchload 22
// //delay(500);
// ////////////////////////////////////////////////////////////
//   move.shoot = false;
//   move.cataEnabled = false;
//   cata.setVelocity(65, percent);
//   intake.setVelocity(65, percent);
//   // was 65 percent
//   cata.spin(forward);
//   intake.spin(reverse);
//   //delay(11250);
//   // was 22.5 seconds
//   // was 11.5 seconds
//   intake.stop();
//   cata.stop();
//   move.cataEnabled = false;
// /////////////////////////////////////////////////////////////
// //delay(1000); //remove
// leftBackWing.set(0);

//   cata.setVelocity(100, percent);
//   cata.spin(forward);


// // through gate align -115.692,  17.431
// move.facePoint(-119.692,  17.431, TKP, TKI, TKD, 12, 8, true);
// move.bto(-119.692,  17.431, 12, 12, YKP, TKP, 12, 5.5);

// // face 180
// move.face(180, TKP, TKI, TKD, 12, 7); 

// // through gate backwards -113.268,  84.715
// move.bto(-115.268,  84.715, 12, 12, YKP, TKP, 12, 5.5);

// // back to -95.863, 107.085
// move.bto( -95.863, 107.085, 12, 12, YKP, TKP, 12, 5.5);

// // pure ram left side goal twice
// cata.stop(coast);
// move.face(270, TKP, TKI, TKD, 12, 5.5);
// leftBackWing.set(1);
//     move.straighth(-25, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(400);
//     move.straighth(14, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 5);
//   leftBackWing.set(0);
//       move.straighth(-25, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(800);
//     move.straighth(6, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 4.5);

// // forward to -85.095,  59.341
// move.to(-85.095,  59.341, 12, 12, YKP, TKP, 12, 5.5);

// // forward to -50.609,  52.604
// move.to(-50.609,  52.604, 12, 12, YKP, TKP, 12, 12);

// // back to -57.060,  85.757
// //move.bto(-57.060,  85.757, 12, 12, YKP, TKP, 12, 5.5); //SWAP
// move.bto(-63.790,  100.403, 12, 12, YKP, TKP*1.2, 0.25, 0);
// delay(2000);

// // forward to -58.476,  63.842
// move.to(-58.476,  63.842, 12, 12, YKP, TKP, 12, 5.5);

// // forward to -34.835,  52.795
// move.to(-34.835,  52.795, 12, 12, YKP, TKP, 12, 12);
// //rightBackWing.set(1);

// // back to -46.239,  85.815
// //move.bto(-46.239,  85.815, 12, 12, YKP, TKP, 12, 5.5); //SWAP
//         move.bto(-52.995,  100.403, 12, 12, YKP, TKP*1.2, 0.25, 0);
//         rightBackWing.set(1);
//         delay(2000);

// // then angle dupes
//  move.to(-59.041,  59.303, 12, 12, YKP, TKP, 12, 9);
//     // bl was 5.5
//     rightBackWing.set(0);

//     // front to -93.504,  56.047 then deploy back wings
//     move.to(-93.504,  56.047, 12, 12, YKP, TKP, 12, 12);
//     // bl was 5.5
//       move.face(230.264, TKP, TKI, TKD, 12, 4); 
//   leftBackWing.set(1);
//   rightBackWing.set(1);
//   move.straighth(-30, 230.264, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(600);
//   move.face(180, TKP, TKI, TKD*1.3, 12, 8);
//   // bl was 4
//   //intake.spin(reverse);
//   move.straighth(-40, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(700);
//   move.straighth(20, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 9);
//   rightBackWing.set(0);
//   leftBackWing.set(0);


//     // forward to -30.164,  46.061 then deploy back wings
//     move.to(-27.164,  59.061, 12, 12, YKP, TKP, 12, 12);
//       move.face(140.264, TKP, TKI, TKD, 12, 4); 
//   leftBackWing.set(1);
//   rightBackWing.set(1);
//   move.straighth(-34, 140.264, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(650);
//   move.face(180, TKP, TKI, TKD*1.3, 12, 8);
//   // bl was 4
//   //intake.spin(reverse);
//   move.straighth(-40, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(700);
//   move.straighth(25, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 9);
//   rightBackWing.set(0);
//   leftBackWing.set(0);



// autonStop();

// }

// void TRSkills(){
//   delay(5000);
//   auto_settings();
//   TKD = 6.548;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);
  
//   TKP = TKP * 1.15;

//   move.cataEnabled = false;

//   // bcurve
//   //move.BCurveTo(12.497,  16.221, 30.312,  17.150, 12, 12, YKP, TKP, 12, 8, 5);

// // first two alliance balls
//   move.bto(-11.478, -18.275 , 12, 12, YKP, TKP, 12, 5.5);
//     move.face(90, TKP, TKI, TKD, 12, 4); 
//     move.straighth(-25, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(400);
//     move.straighth(5, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 1);

// // matchloader align
//     move.to(-15.630, -16.516, 12, 12, YKP, TKP, 12, 4);
//     move.face(339.496, TKP, TKI, TKD, 12, 0); 
//     // was 341.896
//     // was 340.896
//     delay(300);
//     rightBackWing.set(1);

//     //#2
//     // move.to(-19.130, -16.516, 12, 12, YKP, TKP, 12, 5.5);
//     // move.face(350.896, TKP, TKI, TKD, 12, 0); 
//     // rightBackWing.set(1);

// // matchload 22
// //delay(500);
// ////////////////////////////////////////////////////////////
//   move.shoot = false;
//   move.cataEnabled = false;
//   cata.setVelocity(65, percent);
//   intake.setVelocity(65, percent);
//   // was 65 percent
//   cata.spin(forward);
//   intake.spin(reverse);
//   //delay(22500); //RE ENABLE
//   delay(500); //DISABLE
//   // was 22.5 seconds
//   // was 11.5 seconds
//   intake.stop();
//   cata.stop();
//   move.cataEnabled = false;

//   //delay(500); // disable 
// /////////////////////////////////////////////////////////////
// rightBackWing.set(0);


// // forward 3.260,   4.642
// move.bto(1.260, 4.642, 12, 12, YKP, TKP, 12, 5.5);
// // was forward
// cata.setVelocity(100, percent);
// cata.spin(forward);

// // backward 9.785,  76.553
// // move.face(270, TKP, TKI, TKD, 12, 20); 
// // move.face(180, TKP, TKI, TKD, 12, 9);
// move.bto(3.785,  77.553, 12, 12, YKP, TKP*1.2, 12, 7); 
// //leftBackWing.set(1);
// cata.stop(coast);

// rightBackWing.set(1);


// // turn then back to -3.528,  91.415 
// move.bto(-3.528,  93.415, 12, 12, YKP, TKP, 12, 5); 
// leftBackWing.set(1);
// move.face(100, TKP, TKI, TKD, 12, 5.5);
// leftBackWing.set(0);

// move.bto(-17.239, 102.949, 12, 12, YKP, TKP, 12, 0);
// delay(700);

// // side push on right side
// // b curve -11.158, 101.476 and -17.239, 102.949 
// //move.BCurveTo(-11.158, 101.476, -17.239, 102.949, 12, 12, YKP, TKP, 12, 12, 8); 
// //leftBackWing.set(0);
// //look dist was 8
// move.face(90, TKP, TKI, TKD, 12, 5.5);
// rightBackWing.set(1);
//     move.straighth(-25, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(400);
//     move.straighth(6, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 4.5);
//   rightBackWing.set(0);

//   // twice
//   move.to(3.785,  77.553, 8, 12, YKP, TKP*1.2, 12, 14);
//   rightBackWing.set(1);
//   //leftBackWing.set(1);
//   move.BCurveTo(-11.158, 102.476, -18.239, 104.949, 12, 12, YKP, TKP, 12, 12, 8);
//   leftBackWing.set(0);
//   move.face(90, TKP, TKI, TKD, 12, 5.5);
//       move.straighth(-25, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(400);
//     move.straighth(5.5, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 2);
//   rightBackWing.set(0);

  
// autonStop();
// }


 
// void DoubleSkills(){
//   //delay(5000);
//   auto_settings();
//   TKD = 6.548;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);
  
//   TKP = TKP * 1.15;

//   move.cataEnabled = false;

//   // bcurve
//   //move.BCurveTo(12.497,  16.221, 30.312,  17.150, 12, 12, YKP, TKP, 12, 8, 5);

// // first two alliance balls
//   move.bto(-11.478, -18.275 , 12, 12, YKP, TKP, 12, 5.5);
//     move.face(90, TKP, TKI, TKD, 12, 4); 
//     move.straighth(-25, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(400);
//     move.straighth(5, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 1);

// // matchloader align
//     move.to(-15.630, -16.516, 12, 12, YKP, TKP, 12, 4);
//     move.face(339.496, TKP, TKI, TKD, 12, 0);
//     // was 339.496 
//     // was 341.896
//     // was 340.896
//     delay(300);
//     rightBackWing.set(1);

//     //#2
//     // move.to(-19.130, -16.516, 12, 12, YKP, TKP, 12, 5.5);
//     // move.face(350.896, TKP, TKI, TKD, 12, 0); 
//     // rightBackWing.set(1);

// // matchload 22
// //delay(500);
// ////////////////////////////////////////////////////////////
//   move.shoot = false;
//   move.cataEnabled = false;
//   cata.setVelocity(62.5, percent);
//   intake.setVelocity(62.5, percent);
//   // was 65 percent
//   cata.spin(forward);
//   intake.spin(reverse);
//   delay(22500); //RE ENABLE
//   //delay(500); //DISABLE
//   // was 22.5 seconds
//   // was 11.5 seconds
//   intake.stop();
//   cata.stop();
//   move.cataEnabled = false;

//   //delay(500); // disable 
// /////////////////////////////////////////////////////////////
// rightBackWing.set(0);


// // forward 3.260,   4.642
// move.bto(1.260, 4.642, 12, 12, YKP, TKP, 12, 5.5);
// // was forward
// cata.setVelocity(100, percent);
// cata.spin(forward);

// // backward 9.785,  76.553
// // move.face(270, TKP, TKI, TKD, 12, 20); 
// // move.face(180, TKP, TKI, TKD, 12, 9);
// move.bto(3.785,  77.553, 12, 12, YKP, TKP*1.2, 12, 7); 
// //leftBackWing.set(1);
// cata.stop(coast);

// rightBackWing.set(1);


// // turn then back to -3.528,  91.415 
// move.bto(-3.528,  93.415, 12, 12, YKP, TKP, 12, 5); 
// leftBackWing.set(1);
// move.face(100, TKP, TKI, TKD, 12, 5.5);
// leftBackWing.set(0);

// move.bto(-17.239, 102.949, 12, 12, YKP, TKP, 12, 0);
// delay(700);

// // side push on right side
// // b curve -11.158, 101.476 and -17.239, 102.949 
// //move.BCurveTo(-11.158, 101.476, -17.239, 102.949, 12, 12, YKP, TKP, 12, 12, 8); 
// //leftBackWing.set(0);
// //look dist was 8
// move.face(90, TKP, TKI, TKD, 12, 5.5);
// rightBackWing.set(1);
//     move.straighth(-25, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(400);
//     move.straighth(6, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 4.5);
//   rightBackWing.set(0);

//   // twice
//   move.to(3.785,  77.553, 8, 12, YKP, TKP*1.2, 12, 14);
//   rightBackWing.set(1);
//   //leftBackWing.set(1);
//   move.BCurveTo(-11.158, 102.476, -18.239, 104.949, 12, 12, YKP, TKP, 12, 12, 8);
//   leftBackWing.set(0);
//   move.face(90, TKP, TKI, TKD, 12, 5.5);
//       move.straighth(-25, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(400);
//     move.straighth(5.5, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 2);
//   rightBackWing.set(0);

//   // move.face(0, TKP*1.2, TKI, TKD, 12, 5.5);
//   // rightBackWing.set(1);


//   // //  curve -19.393,  84.189 and -24.343,  75.711  
//   // move.BCurveTo(-19.393,  84.189, -24.343,  75.711, 12, 12, YKP, TKP, 12, 5, 8); 

//   // // to -36.518,  74.329  
//   // // to -43.093,  83.955
//   // //move.BCurveTo(-36.518,  74.329, -43.093,  83.955, 12, 12, YKP, TKP, 12, 5, 8);
//   // move.bto(-36.518,  74.329, 12, 12, YKP, TKP, 12, 7);
//   // rightBackWing.set(0);
//   // move.bto(-43.093,  83.955, 12, 12, YKP, TKP, 12, 7); 
//   // rightBackWing.set(0);

//   //   move.face(180, TKP*1.2, TKI, TKD, 12, 5.5);
//   //   move.straighth(-25, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   //   delay(400);
//   //   move.straighth(5.5, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 2);  

// // re work *******************************************************************
//     // front pushes
//     // forward to -23.991,  53.920
//     //rightWing.set(1);
//         move.to(-25.991,  56.320, 12, 12, YKP, TKP, 12, 12);
//       rightWing.set(0);
//     // forward to -47.110,  51.437
//         move.to(-50.110,  56.437, 12, 12, YKP, TKP, 12, 5.5);
//         move.face(0, TKP*1.2, TKI, TKD, 12, 5.5);
//         //rightWing.set(1);
//         // was 180
//         // leftWing.set(1);
//         // rightWing.set(1);
//     // backward to -44.995,  86.403 with no bl (delay) and elevate y  
//         move.to(-52.995,  104.403, 12, 12, YKP, TKP*1.2, 1, 0);
//         // was backward
//         //leftBackWing.set(1);
//         delay(1500);
//         rightWing.set(0);
//         leftWing.set(0);
//         // was 1200

//     // second front push
//     // forward -46.392,  59.700
//     move.bto(-54.392,  59.700, 12, 12, YKP, TKP, 12, 12);
//     // was forward
//     //leftBackWing.set(0);
//     // forward -60.584,  53.377
//     move.bto(-64.584,  56.377, 12, 12, YKP, TKP, 12, 5.5);
//     // was forward
//     //move.face(180, TKP, TKI, TKD, 12, 5.5);
//     // backward to -58.290,  85.128 with no bl (delay) and elevate y 
//     move.to(-63.790,  100.403, 12, 12, YKP, TKP*1.2, 0.25, 0);
//     // was backward
//     rightWing.set(1);
//     //rightWing.set(1);
//     delay(1500);
//     rightWing.set(0);
//     // was 1400

//     // third front push
//     // forward -63.137,  57.651
//     //     move.to(-63.137,  85.651, 12, 12, YKP, TKP, 12, 12);
//     // leftBackWing.set(0);
//     // CANCEL
//     // // forward -77.434,  52.359 
//     //     move.to(-77.434,  52.359, 12, 12, YKP, TKP, 12, 5.5);
//     // //  backward -75.993, old y with boost and stuff 
//     //     move.bto(-75.993,  95.403, 12, 12, YKP, TKP*1.2, 1, 0);
//     // leftBackWing.set(1);
//     // delay(1800);
//     // // was 1200

//     //////////////////////////////////////////////////////// DUPE ANGLE
//     // front to -59.041,  59.303
//     move.bto(-59.041,  59.303, 12, 12, YKP, TKP, 12, 9);
//     //leftWing.set(1);
//     // was forward
//     // bl was 5.5
//     leftBackWing.set(0);
//     rightWing.set(0);
//     rightBackWing.set(1);

//     // front to -93.504,  56.047 then deploy back wings
//     move.bto(-93.504,  56.047, 12, 12, YKP, TKP, 12, 12);
//     // was forward
//     //rightBackWing.set(1);
//     //rightWing.set(1);
//     move.face(180, TKP, TKI, TKD, 12, 20);
//     move.face(236, TKP, TKI, TKD, 12, 8);
//     leftWing.set(0);
//     rightWing.set(0);
//     //rightBackWing.set(0);
//     // bl was 5.5
//       move.face(230.264, TKP, TKI, TKD, 12, 4); 
//   leftBackWing.set(1);
//   rightBackWing.set(1);
//   move.straighth(-30, 230.264, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(500);
//   move.face(180, TKP, TKI, TKD*1.3, 12, 8);
//   // bl was 4
//   //intake.spin(reverse);
//   move.straighth(-40, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(700);
//   move.straighth(17, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 9);
//   // was 20 units
//   rightBackWing.set(0);
//   leftBackWing.set(0);


//     // forward to -30.164,  46.061 then deploy back wings
//     move.to(-25.164,  57.061, 12, 12, YKP, TKP, 12, 12);
//       move.face(140.264, TKP, TKI, TKD, 12, 4); 
//   leftBackWing.set(1);
//   rightBackWing.set(1);
//   move.straighth(-34, 140.264, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(550);
//   move.face(180, TKP, TKI, TKD*1.3, 12, 8);
//   // bl was 4
//   //intake.spin(reverse);
//   move.straighth(-40, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(700);
//   move.straighth(17, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 9);
//   rightBackWing.set(0);
//   leftBackWing.set(0);

// // forward to -117.572,  78.829
// move.face(270, TKP, TKI, TKD, 12, 5.5);
// // was 265
// move.to(-115.572,  79.829, 12, 12, YKP, TKP, 12, 12);

// // OLD STUFF FOR GROUPING SIDE
// // leftWing.set(1);

// // // turn then to -116.403,  88.722
// // move.face(0, TKP, TKI, TKD, 12, 12);
// // move.face(28, TKP, TKI, TKD, 12, 5.5);
// // leftWing.set(0);
// /////////////////////////////////////

// //move.to(-116.403,  88.722, 12, 12, YKP, TKP, 12, 12);

// // back to -118.456,  80.137
// //move.bto(-118.456,  80.137, 12, 12, YKP, TKP, 12, 12);

// // turn then back to -113.052,  95.680 
// //move.face(90, TKP, TKI, TKD, 12, 12);
// //leftWing.set(1);
// move.face(0, TKP, TKI, TKD, 12, 5.5);
// leftWing.set(0);
// // was 180
// //leftBackWing.set(1);
// rightWing.set(1);
// // move.bto(-113.052,  100.680 , 12, 12, YKP, TKP, 12, 12);

// // // back to -101.735, 106.465
// // move.bto(-97.735, 106.465, 12, 12, YKP, TKP, 12, 0);
// // delay(800);
// move.CurveTo(-99.052,  100.680, -87.735, 107.965, 12, 12, YKP, TKP, 12, 12, 8); 
// // was Backwards

// // face then pure ram backwards with back left wing down
// move.face(90, TKP, TKI, TKD, 12, 5.5);
// // was 270
// //leftBackWing.set(1);
//     move.straighth(35, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(700);
//     move.straighth(-11, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 4.5);
//     move.straighth(35, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(400);
//     move.straighth(-11, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 4.5);
//     rightWing.set(0);
//     move.face(270, TKP, TKI, TKD, 12, 5.5);
//     leftBackWing.set(1);
//     move.straighth(-35, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(400);
//     move.straighth(6, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 4.5);
//   leftBackWing.set(0);

//   BarHang.set(1);
//   move.to(-116.572,  78.829, 12, 12, YKP, TKP, 12, 12);
//   move.face(180, TKP, TKI, TKD, 12, 5.5);
//   move.straighth(100, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(1150);
//   BarHang.set(0);
//   PistonWheel.set(1);


// leftBackWing.set(0);


//   autonStop();

// cata.setVelocity(100, percent);
// cata.spin(forward);

// ///////////////////////////////////// CLOSE SIDE SWEEP RIGHT GATE
// move.to(6.285,  84.553, 12, 12, YKP, TKP*1.2, 12, 7); 
// move.face(180, TKP, TKI, TKD*1.3, 12, 4);
// move.to(2.260, -2.642, 12, 12, YKP, TKP, 12, 9);

// // // f to -24.255,   3.121
//   move.facePoint(-24.255,   3.121, TKP, TKI, TKD, 12, 9, false);
//   move.to(-24.255,   3.121, 12, 12, YKP, TKP, 12, 5.5);

// // delay(200); //settle

// // // f to -28.743,  13.913
//   move.facePoint(-24.743,  18.913, TKP, TKI, TKD, 12, 9, false);
//   move.to(-24.743, 18.913, 12, 12, YKP, TKP*1.2, 12, 5.5);

// //move.CurveTo(-24.255,   3.121, -28.743,  14.913, 12, 12, YKP, TKP, 12, 14, 8); 

// leftWing.set(1);
// rightWing.set(1);

// move.swingOnLeft(270, TKP, TKI, TKD, 12, 5.5);

// move.straighth(95, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
// delay(1600);

// move.straighth(-10, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 6.5);

// leftWing.set(0);
// rightWing.set(0);

// // b to -89.687,  13.647
//   move.facePoint(-118.687,  -4.647, TKP, TKI, TKD, 12, 9, true);
// move.bto(-118.687,  -4.647, 12, 12, YKP, TKP*1.4, 12, 9);

// // f to -125.071,  12.913
// move.to(-125.071,  12.913, 12, 12, YKP, TKP, 12, 9);

// BarHang.set(1);

// move.face(0, TKP, TKI, TKD*1.3, 12, 5.5);
// rightWing.set(1);

// move.straighth(90, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
// delay(700);
// leftWing.set(0);
// rightWing.set(0);

// BarHang.set(0);

// autonStop();
// ////////////////////////////////////////////////////////////////

// //////////////////////////////////// ALTERNATE WITH CLOSE SIDE SWEEP ENTER ON LEFT GATE

//   // gate crossing

//   // back to -120.761,  80.870
//   move.bto(-115.761, 78.870, 12, 12, YKP*0.95, TKP, 12, 5.5);

//   delay(200);
//   //settle

//   // back to -121.207,   5.067
//   move.bto(-118.207, -4.067, 12, 12, YKP, TKP, 12, 5.5);
//   cata.stop(coast);

//   // forward to -98.646,   9.743 
//   move.to(-98.646, 0.743, 12, 12, YKP, TKP*0.9, 12, 12);

//   delay(200);
//   //settle


//   move.to(-92.629,  12.682, 12, 12, YKP, TKP, 12, 5.5);

//         move.face(0, TKP, TKI, TKD, 12, 4); 

//         leftWing.set(1);
//         rightWing.set(1);

//   move.swingOnRight(90, TKP, TKI, TKD, 12, 5.5);

//   // straight to -21.582,  18.259
//     move.straighthToPoint(95, -21.582,  18.259, YKP, YKD*1.3, TKP, TKD, 12, 12, 12, 0, false);
//     delay(2500);

//     leftWing.set(0);
//     rightWing.set(0);











  




//   autonStop();




//   /////////////////////////////////////////////////////////////////////

//   // second right side push
//   //leftWing.set(1);
//     move.to(-0.295,  77.860, 12, 12, YKP, TKP, 12, 5.5);
//   rightBackWing.set(1);
//     move.BCurveTo(-11.158, 102.476, -18.239, 104.949, 12, 12, YKP, TKP, 12, 12, 8);
//   move.face(90, TKP, TKI, TKD, 12, 5.5);
//   rightBackWing.set(1);
//       move.straighth(-25, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(400);
//     move.straighth(5.5, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 2);
//   rightBackWing.set(0);
//         move.straighth(-25, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(400);
//     move.straighth(5.5, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 2);

//     /////////////////////////////////////////////////////////////////////////

//     // going to hang
//     // forward to -50.887,  71.260
//     // move.to(-50.887,  77.260, 12, 12, YKP, TKP, 12, 8);
//     leftBackWing.set(0);
//     rightBackWing.set(0);

//     // forward to -2.295,  69.860
//     move.to(-0.295,  73.860, 12, 12, YKP, TKP, 12, 5.5);
//     BarHang.set(1);
//     // forward to -4.780,  29.619 
//     move.to(-0.780,  2.619 , 12, 12, YKP, TKP, 12, 0); 
//     delay(1800);  
//     BarHang.set(0);
// // **********************************************************************************

// ///////////////////////////////////////////////////////////////////////////////////////
//  autonStop();

//     cata.setVelocity(100, percent);
//     cata.spin(forward);

//     // gate through tween: -87.694,  69.784 and -121.521,  83.789 
//     //move.CurveTo(-87.694,  69.784, -126.921,  83.789, 12, 12, YKP, TKP, 12, 12, 8);
//     // SWAP
//     move.to(-87.694,  76.784, 12, 12, YKP, TKP, 12, 12);
//     leftBackWing.set(0);
//     move.to(-126.921,  85.789, 12, 12, YKP, TKP, 12, 5.5);
//     // look dist was 8
//     // move.face(270, TKP, TKI, TKD, 12, 13); 
//     move.face(0, TKP, TKI, TKD, 12, 18);

//     //align second loader
//     // forward to -122.684,   7.924
//     move.bto(-122.684,   7.924, 12, 12, YKP, TKP, 12, 12);
//     cata.stop(coast);
//     // bl was 5.5
//     // backward to -106.320, -15.864 
//     move.bto(-110.308, -19.847, 12, 12, YKP, TKP, 12, 5.5);
//     //matchload align
//     move.face(19.420, TKP, TKI, TKD, 12, 0); 
//     leftBackWing.set(1);

//     // matchload 22 #2
// //delay(500);
// ////////////////////////////////////////////////////////////
//   move.shoot = false;
//   move.cataEnabled = false;
//   cata.setVelocity(65, percent);
//   intake.setVelocity(65,percent);
//   cata.spin(forward);
//   intake.spin(reverse);
//   delay(11500);
//   intake.stop();
//   cata.stop();
//   move.cataEnabled = false;
// /////////////////////////////////////////////////////////////
//     leftBackWing.set(0);

//     cata.setVelocity(100, percent);
//     cata.spin(forward);

//     // back to -121.707,  15.209
//     move.bto(-121.707,  15.209, 12, 12, YKP, TKP, 12, 5.5);
//     // back curve -120.889,  78.889 and -101.125, 101.108  
//     move.bto(-120.889,  80.889, 12, 12, YKP, TKP, 12, 12); 
//     rightBackWing.set(1);
//     move.BCurveTo(-120.889,  80.889, -99.125, 104.108, 12, 12, YKP, TKP, 12, 12, 8); 
//     rightBackWing.set(0);
//     cata.stop(coast);
//     move.face(270, TKP, TKI, TKD, 12, 5.5);
//     leftBackWing.set(1);
//     move.straighth(-25, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(400);
//     move.straighth(10, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 4.5);
//     leftBackWing.set(0);
//     move.straighth(-25, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(400);
//     move.straighth(4.5, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 2); 
// ///////////////////////////////////////////////////////////////

//     // forward to -96.889,  53.679
//     move.to(-92.889,  55.679, 12, 12, YKP, TKP, 12, 12);

//     // forward to -58.419,  53.631
//     move.to(-58.419,  53.631, 12, 12, YKP, TKP, 12, 5.5);

//     // back to with no delay boosted y -57.553,  83.526
//     move.bto(-57.553,  94.526, 12, 12, YKP, TKP, 12, 0);
//     delay(1000);

//     // front to -59.041,  59.303
//     move.to(-59.041,  59.303, 12, 12, YKP, TKP, 12, 5.5);

//     // front to -93.504,  56.047 then deploy back wings
//     move.to(-93.504,  56.047, 12, 12, YKP, TKP, 12, 5.5);
//     leftBackWing.set(1);
//     rightBackWing.set(1);

//     // back to -68.624,  80.200 
//     move.bto(-68.624,  80.200, 12, 12, YKP, TKP, 12, 5.5);

//     // back to -67.469,  84.492 with no delay boosted y
//     move.bto(-67.469,  94.492, 12, 12, YKP, TKP, 12, 0); 
//     delay(700);

//     // forward to -66.925,  53.551
//     move.to(-66.925,  53.551, 12, 12, YKP, TKP, 12, 12);
//     leftBackWing.set(0);
//     rightBackWing.set(0);

//     // forward to -30.164,  46.061 then deploy back wings
//     move.to(-30.164,  46.061, 12, 12, YKP, TKP, 12, 12);
//     leftBackWing.set(1);
//     rightBackWing.set(1);

//     // back to -45.370,  63.760
//     move.bto(-45.370,  63.760, 12, 12, YKP, TKP, 12, 12);
//     // back to with no delay boosted y -52.639,  80.810
//     move.bto(-52.639,  95.810, 12, 12, YKP, TKP, 12, 0);
//     delay(700);

//     // going to hang
//     // forward to -50.887,  71.260
//     move.to(-50.887,  71.260, 12, 12, YKP, TKP, 12, 8);
//     leftBackWing.set(0);
//     rightBackWing.set(0);

//     // forward to -2.295,  69.860
//     move.to(-2.295,  73.860, 12, 12, YKP, TKP, 12, 5.5);
//     BarHang.set(1);
//     // forward to -4.780,  29.619 
//     move.to(-4.780,  29.619 , 12, 12, YKP, TKP, 12, 0); 
//     delay(600);  

//     autonStop();





    

  



//     autonStop();



// }

// void NewWashington(){
//   delay(5000);
//   auto_settings();
//    // move.facePoint(0, -24, TKP*1.3, TKI, TKD, 12, 0, true);
//   //delay(1000000);
//   TKD = 6.548;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);
  
//   TKP = TKP * 1.15;

//   move.cataEnabled = false;

//   // forward to 10.516,  17.636
//   move.swingOnRighttoPoint(10.516,  17.636, TKP, TKI, TKD, 12, 3, false);
//   // bl was 20
//   move.straighthToPoint(16, 10.516,  18.636, YKP, YKD*1.3, TKP, TKD, 12, 12, 12, 3, false);
//   // bl was 8
//   // forward to 27.950,  17.596
//   move.swingOnRighttoPoint(27.950,  18.596, TKP, TKI, TKD, 12, 8, false);
//   move.straighthToPoint(20, 35.950,  18.596, YKP*1.5, YKD, TKP, TKD, 12, 12, 12, 0, false);
//   delay(700);
//   // back to 16.672,  17.443
//   move.straighthToPoint(-4.5, 16.672,  17.443, YKP, YKD, TKP, TKD, 12, 12, 12, 0.5, true);
//   //bl was 8

//     //aiming
//     move.face(160.896, TKP, TKI, TKD, 12, 0); 
//     delay(200);

//     move.straighth(-3, 160.896, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//     delay(400);

//     move.face(160.896, TKP, TKI, TKD, 12, 0); 
//     rightBackWing.set(1);

// ////////////////////////////////////////////////////////////

//   move.shoot = false;
//   move.cataEnabled = false;
//   cata.setVelocity(65, percent); //re enable this
//   intake.setVelocity(65,percent);
//   // was 60 
//   cata.spin(forward);
//   intake.spin(reverse);
//   delay(22000);
//   // was 22,000
//   // was 28,500
//   // was 27,800
//   // was 32,800
//   intake.stop();
//   cata.stop();
//   move.cataEnabled = false;

//  // delay(500);


// /////////////////////////////////////////////////////////////
    
//     rightBackWing.set(0);

//   //normal turn then to 16.573, -11.240
//   move.facePoint(18.573, -11.240, TKP, TKI, TKD, 12, 3, false);
//   move.shoot = false;
//   move.cataEnabled = false;
//   cata.setStopping(hold);
//   cata.setVelocity(100, percent);
//   intake.setVelocity(100, percent);
//   cata.spinFor(700, degrees, false);
//   intake.spinFor(-700, degrees, false);
//   // bl was 15
//   move.to(18.573, -11.240, 12, 12, YKP, TKP, 12, 3);

//   //swing on right then to 17.237, -24.635 
//   move.swingOnRighttoPoint(18.237, -24.635, TKP, TKI, TKD, 12, 3, false);
//   // bl was 20
//   move.straighthToPoint(13, 18.237, -24.635, YKP, YKD, TKP, TKD, 12, 12, 12, 4, false);

//   rightBackWing.set(1);

//   move.face(90, TKP, TKI, TKD, 12, 5);
//   rightBackWing.set(1); 
//   leftWing.set(1);
//   // cata.stop(coast);
//   // intake.stop(coast);

//   //straight to 86.235, -31.416
//   move.straighth(10, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 5);
//   move.straighth(-10, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 5);
//   rightWing.set(1);
//   rightBackWing.set(0);
//   move.facePoint(86.235, -29.416, TKP, TKI, TKD, 12, 3, false);
//   move.straighthToPoint(63, 86.235, -27.416, YKP, YKD, TKP, TKD, 12, 12, 12, 5, false);
//   // was 67 with bl 8

//   move.swingOnRight(180, TKP*1.5, TKI, TKD, 12, 0);
//   delay(500);
//   // was bl 5

//   move.straight(25, YKP, YKD, 12, 12, 0);
//   delay(600);

//   leftWing.set(0);
//   rightWing.set(0);

//   //back to 88.913, -16.362
//   move.facePoint(88.913, 3.362, TKP, TKI, TKD, 12, 6, true);
//   // bl was 15
//   move.bto(88.913, 3.362, 12, 12, YKP, TKP, 12, 3);
//   // back to 109.034,  -0.133
//   // // turn to 0
//   //delay(200);
//   //settle

//   // to 116.799, -14.811
//   move.facePoint(118.799, -14.811, TKP*1.2, TKI, TKD, 12, 6, false);
//   // bl was 15
//   //move.to(118.799, -14.811, 12, 12, YKP, TKP, 12, 3);
//   move.straighthToPoint(30, 118.799, -14.811, YKP, YKD, TKP, TKD, 12, 12, 12, 5, false);

//   // face 180
//   move.face(180, TKP, TKI, TKD, 7, 5); 

//   // to 114.350, -89.085
//   move.facePoint(116.350, -84.085, TKP, TKI, TKD, 12, 6, false);
//   // bl was 15
//   move.to(116.350, -84.085, 12, 12, YKP, TKP, 12, 8);

//   // to 100.436, -106.362
//   move.swingOnRighttoPoint(93.436, -104.362, TKP, TKI, TKD, 12, 6, false);
//   // bl was 15
//   move.to(93.436, -104.362, 12, 12, YKP, TKP, 12, 5);

//   //straighth at 270
//   //move.swingOnRighttoPoint(81.177, -106.584, TKP, TKI, TKD, 12, 20, false);
//   move.swingOnRight(270, TKP, TKI, TKD, 12, 5);
//   rightWing.set(1);
//   //move.straighthToPoint(35, 81.177, -106.584, YKP, YKD, TKP, TKD, 12, 12, 12, 0, false);
//   move.straighth(40, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(500);
//   move.straighth(-15, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 5);
//   rightWing.set(0);
//   delay(100);
//   move.straighth(40, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(500);
//   move.straighth(-15, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 5);
//   delay(100);

//   rightWing.set(0);

//   // to 86.013, -104.673
//   // move.facePoint(89.013, -104.673, TKP, TKI, TKD, 12, 6, false);
//   // move.to(89.013, -104.673, 12, 12, YKP, TKP, 12, 3);
//   // // to 87.882, -89.743
//   // move.facePoint(89.882, -89.743, TKP, TKI, TKD, 12, 6, true);
//   // move.bto(89.882, -89.743, 12, 12, YKP, TKP, 12, 3);
//   // back turn deploy wings then back to 87.056, -64.368 
//   move.facePoint(85.056, -64.368 , TKP, TKI, TKD, 12, 6, true);
//   // leftBackWing.set(1);
//   // rightBackWing.set(1);
//   move.bto(85.056, -64.368, 12, 12, YKP, TKP, 12, 3);

//   //swing on right to face 90
//   move.swingOnRight(90, TKP, TKI, TKD, 12, 5);
//   leftBackWing.set(0);
//   rightBackWing.set(0);

//   //move.straighth(-25, 90, YKP, YKD, TKP, TKD, 12, 12, 1, 4);
//   // change to coord: 58.246, -58.385
//   move.facePoint(55.246, -58.385 , TKP, TKI, TKD, 12, 6, true);
//   move.bto(55.246, -58.385, 12, 12, YKP, TKP, 12, 3);


//   //move.face(90, TKP*1.2, TKI, TKD*1.3, 12, 8);
//   move.face(0, TKP*1.2, TKI, TKD*1.3, 12, 4);
//   //IN CASE SAVE
//   move.straighth(-50, 0, YKP, YKD, TKP, TKD, 12, 12, 1, 0);
//   delay(800);
//   move.straighth(27.5, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 6);
//   delay(50);

//   // swing left to face 270
//   move.swingOnLeft(270, TKP, TKI, TKD, 12, 5);

//   // back to 86.320, -54.564 
//   move.facePoint(86.320, -54.564, TKP, TKI, TKD, 12, 6, true);
//   move.bto(86.320, -54.564, 12, 12, YKP, TKP, 12, 3);

//   // angled push
//   //move.swingOnLeft(230.264, TKP, TKI, TKD, 12, 5);
//   move.face(50.265, TKP*1.2, TKI, TKD*1.3, 12, 4);
//   leftBackWing.set(1);
//   rightBackWing.set(1);
//   move.straighth(-30, 50.264, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(600);
//   move.face(0, TKP, TKI, TKD*1.3, 12, 4);
//   move.straighth(-40, 0, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(700);
//   move.straighth(27.5, 0, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 6);
//   leftBackWing.set(0);
//   rightBackWing.set(0);

//   // swing on left to face 270
//   move.swingOnRight(90, TKP, TKI, TKD, 12, 5);
//   // to 45.381, -56.020
//   move.facePoint(43.681, -56.020, TKP, TKI, TKD, 12, 6, true);
//   move.bto(43.681, -56.020, 12, 12, YKP, TKP, 12, 3);

//   // ram
//   move.face(0, TKP*1.2, TKI, TKD*1.3, 12, 3);
//   move.straighth(-50, 0, YKP, YKD, TKP, TKD, 12, 12, 1, 0);
//   delay(800);
//   move.straighth(27.5, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 3.2);
//   delay(50);

//   // swing on right to face 90
//   move.swingOnRight(90, TKP, TKI, TKD, 12, 5);

//   // back to  16.835, -58.377
//   move.facePoint(16.835, -58.377, TKP, TKI, TKD, 12, 6, true);
//   move.bto(16.835, -58.377, 12, 12, YKP, TKP, 12, 3);

//   // angled push
//   move.face(307.616, TKP, TKI, TKD, 12, 4); 
//   leftBackWing.set(1);
//   rightBackWing.set(1);
//   move.straighth(-27, 307.616, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(400);
//   move.face(0, TKP, TKI, TKD*1.3, 12, 4);
//   move.straighth(-40, 0, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(800);
//   move.straighth(10, 0, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 3);
//   rightBackWing.set(0);
//   leftBackWing.set(0);



//   autonStop();






// /////////////////////////////////////////////////////////////////////////////////
  
//   //GONNA REDO:

// // to  84.766, -70.357
//   move.facePoint(83.766, -58.357, TKP, TKI, TKD, 12, 6, false);
//   // bl was 15
//   move.to(83.766, -58.357, 12, 12, YKP, TKP, 12, 3);

// // swing on left to 270 then 215 with front wings out
//   move.swingOnLeft(270, TKP*1.2, TKI, TKD*1.3, 12, 4);
//   move.straighth(13, 270, YKP, YKD, TKP, TKD, 12, 12, 1, 4);

//   //old corral
//   // leftWing.set(1);
//   // delay(300);
//   // move.swingOnLeft(230, TKP, TKI, TKD, 12, 4);
//   // rightWing.set(1);
//   // move.straighth(14, 230, YKP, YKD, TKP, TKD, 12, 12, 12, 5);
//   // leftWing.set(0);
//   // rightWing.set(0);

//   move.face(270, TKP*1.2, TKI, TKD*1.3, 12, 8);
//   move.face(0, TKP*1.2, TKI, TKD*1.3, 12, 4);
//   //IN CASE SAVE
//   move.straighth(-50, 0, YKP, YKD, TKP, TKD, 12, 12, 1, 0);
//   delay(800);
//   move.straighth(27.5, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 6);
//   delay(50);
//   // move.straighth(-50, 0, YKP, YKD, TKP, TKD, 12, 12, 1, 0);
//   // delay(1000);
//   // move.straighth(10, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 0.5);
//   // delay(50);
//   // leftBackWing.set(1);
//   // move.straighth(-50, 0, YKP, YKD, TKP, TKD, 12, 12, 1, 0);
//   // delay(1000);
//   // move.straighth(8, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 0.5);
//   // delay(50);
//   // leftBackWing.set(0);
//   // move.straighth(19.5, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 0.5);
//   // delay(50);

//   move.swingOnLeft(270, TKP, TKI, TKD, 12, 5);
//   //move.straighthToPoint(35, 81.177, -106.584, YKP, YKD, TKP, TKD, 12, 12, 12, 0, false);
//   move.straighth(12, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 5);

//   move.face(0, TKP*1.2, TKI, TKD*1.3, 12, 4);
//   rightBackWing.set(1);
//   move.straighth(-50, 0, YKP, YKD, TKP, TKD, 12, 12, 1, 0);
//   delay(1000);
//   move.straighth(27.5, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 6);
//   delay(50);
//   rightBackWing.set(0);

//   //dupe (3rd straight)
//   move.swingOnLeft(270, TKP, TKI, TKD, 12, 5);
//   move.straighth(12, 270, YKP, YKD, TKP, TKD, 12, 12, 12, 5);
//   move.face(0, TKP*1.2, TKI, TKD*1.3, 12, 4);
//   rightBackWing.set(1);
//   move.straighth(-50, 0, YKP, YKD, TKP, TKD, 12, 12, 1, 0);
//   delay(1000);
//   move.straighth(27.5, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 6);
//   delay(50);
//   rightBackWing.set(0);


//   autonStop();
// }

// void WashingtonSkills(){
//   delay(5000);
//   auto_settings();
//    // move.facePoint(0, -24, TKP*1.3, TKI, TKD, 12, 0, true);
//   //delay(1000000);
//   TKD = 5.8;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);

//   move.cataEnabled = false;

//   // turn on later
//   intakePull.set(0);
//   intake.setVelocity(100, percent);
//   //delay(100);
//   move.cataEnabled = false;
//   TKP = TKP*1.15;

//   // example 
//   /*
//   move.facePoint(12.497,  16.221, TKP, TKI, TKD, 12, 15, false);
//   move.to(12.497,  16.221, 12, 12, YKP, TKP, 12, 3);
//   move.facePoint(30.312,  17.150, TKP, TKI, TKD*1.2, 12, 15, false);
//   move.to(30.312,  17.150, 12, 12, YKP, TKP, 12, 0);
//   delay(300);
//   */

// // forward to  16.726,  17.621
//   move.facePoint(16.726,  16.621, TKP, TKI, TKD, 12, 15, false);
//   move.to(16.726,  16.621, 12, 12, YKP, TKP, 12, 3);
// // forward to 27.902,  18.295
//   move.facePoint(27.902,  18.295, TKP, TKI, TKD, 12, 15, false);
//   move.to(27.902,  18.295, 12, 12, YKP, TKP, 12, 3);
// // back to 18.549,  18.034
//   move.facePoint(18.549,  18.034, TKP, TKI, TKD, 12, 15, true);
//   move.bto(18.549,  18.034, 12, 12, YKP, TKP, 12, 3);
// // face 160.936
//         move.face(160.396, TKP*1.2, TKI, TKD*1.2, 12, 0); 
//         // was -17.396
//         // was -12.395
//         // was h -18.935
//         delay(200);
//         rightBackWing.set(1);
//         delay(500);
//         rightBackWing.set(1);

// // forward to 19.480,  -8.187
//   move.facePoint(19.480,  -8.187, TKP, TKI, TKD, 12, 15, false);
//   move.to(19.480,  -8.187, 12, 12, YKP, TKP, 12, 3);
// // forward to 19.033, -26.016
//   move.facePoint(19.033, -21.516, TKP, TKI, TKD, 12, 15, false);
//   move.to(19.033, -21.516, 12, 12, YKP, TKP, 12, 3);
// // face 90
// move.face(90, TKP*1.2, TKI, TKD*1.2, 12, 4); 
// // drop back right wing
// //rightBackWing.set(1);
// // forward to  27.757, -25.726 then back to 25.340, -23.403 
//   //move.facePoint(27.757, -24.726, TKP, TKI, TKD, 12, 15, false);
//   //move.to(27.757, -24.726, 12, 12, YKP, TKP, 12, 3);
//   move.straighth(24, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 5);
//   rightBackWing.set(0);

//   // back to 
//   move.facePoint(25.340, -23.403, TKP, TKI, TKD, 12, 15, true);
//   move.bto(25.340, -23.403, 12, 12, YKP, TKP, 12, 3);


// // forward to 95.099, -26.334
// rightBackWing.set(0);
// rightWing.set(1);
// leftWing.set(1);
//   //move.facePoint(95.099, -26.334, TKP, TKI, TKD, 12, 15, false);
//   //move.to(95.099, -26.334, 10, 12, YKP, TKP, 1, 3);
//   move.face(90, TKP*1.2, TKI, TKD*1.2, 12, 4); 
//   move.straighth(90, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 1, 0);
//   delay(1800);
//   move.straighth(0, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 8, 12, 1, 4);
//   delay(200);
//   move.straighth(-12, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 8, 12, 12, 5);
// cata.setStopping(hold);
// cata.setVelocity(100, percent);
// intake.setVelocity(100, percent);
// cata.spinFor(700, degrees, false);
// intake.spinFor(-700, degrees, false);
// rightWing.set(0);
// leftWing.set(0);

// // forward to 112.210,   1.903
//   move.facePoint(112.210,   1.903, TKP, TKI, TKD, 12, 15, false);
//   move.to(112.210,   1.903, 12, 12, YKP, TKP, 12, 3);
//   leftBackWing.set(1);
// // face 180 then 220
//   move.face(180, TKP*1.2, TKI, TKD*1.2, 12, 9); 
//     move.face(220, TKP*1.2, TKI, TKD*1.2, 12, 4); 
//       //leftBackWing.set(0);
// // forward to 116.889, -22.989
//   move.facePoint(116.889, -17.989, TKP*1.2, TKI, TKD*1.2, 12, 15, true);
//   move.bto(116.889, -17.989, 12, 12, YKP, TKP, 12, 3);



// // back to 115.387, -33.999
//   move.facePoint(115.387, -34.999, TKP, TKI, TKD, 12, 15, true);
//   move.bto(115.387, -34.999, 12, 12, YKP, TKP, 12, 3);
// // wing up
// leftBackWing.set(0);
// // back by 10 
//     move.straighth(-15, 0, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 1, 5);
// // wing down
// leftBackWing.set(1);
// // back to 114.717, -84.095
//   move.facePoint(112.717, -84.095, TKP, TKI, TKD, 12, 15, true);
//   move.bto(112.717, -84.095, 12, 12, YKP, TKP, 12, 3);


// //going forward: (old)
// // // forward to 109.288, -91.245
// // //rightWing.set(1);
// //   move.facePoint(117.288, -91.245, TKP, TKI, TKD, 12, 15, false);
// //   move.to(117.288, -91.245, 8, 12, YKP, TKP, 12, 7);
  
// // forward to 97.762, -106.661
//   move.facePoint(95.762, -102.961, TKP, TKI, TKD, 12, 15, true);
//   move.bto(95.762, -102.961, 8, 12, YKP, TKP, 12, 3);

// // pure ram at 270
// move.face(90, TKP, TKI, TKD, 12, 4);
// //rightWing.set(1);

//   move.straighth(-24, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(600);
//   move.straighth(13, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 8);
//   delay(100);
//   leftBackWing.set(0);
//   //intake.spin(reverse);
//   move.straighth(-20, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(600);
//   move.straighth(10, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 2);
//   //intake.spin(forward);

// //forward to 91.605, -65.264
//   move.facePoint(87.605, -65.264, TKP, TKI, TKD, 12, 15, false);
//   move.to(87.605, -65.264, 12, 12, YKP, TKP, 12, 3);

// // back to with both back wings down 84.769, -65.608
//   //move.facePoint(78.769, -65.608, TKP, TKI, TKD, 12, 15, true);
//   move.face(90, TKP, TKI, TKD, 12, 4);
//   leftBackWing.set(1);
// rightBackWing.set(1);
//   //move.bto(78.769, -65.608, 12, 12, YKP, TKP, 12, 3);
//   move.straighth(-10, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 4);
// rightBackWing.set(0);
// // back to with left back wing down 73.210, -65.941
//   // move.facePoint(69.210, -65.941, TKP, TKI, TKD, 12, 15, true);
//   // move.bto(69.210, -65.941, 12, 12, YKP, TKP, 12, 3);
//   move.straighth(-15, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 4);
// leftBackWing.set(0);
// // pure ram at 180


// autonStop();

// }

// void GordProgSkills(){
//   //delay(5000);
//   auto_settings();
//    // move.facePoint(0, -24, TKP*1.3, TKI, TKD, 12, 0, true);
//   //delay(1000000);
//   TKD = 5.8;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);

//   move.cataEnabled = false;

//   // turn on later
//   intakePull.set(0);
//   intake.setVelocity(100, percent);
//   //delay(100);
//   move.cataEnabled = false;
//   TKP = TKP*1.15;

// ////////////////////////////////////////////////////////////////////////////////////REDO AGAIN^

// // curves:  12.497,  17.221 and 25.312,  19.150 (with boosted x)
//   move.facePoint(12.497,  16.221, TKP, TKI, TKD, 12, 15, false);
//   move.to(12.497,  16.221, 12, 12, YKP, TKP, 12, 3);
//   move.facePoint(30.312,  17.150, TKP, TKI, TKD*1.2, 12, 15, false);
//   move.to(30.312,  17.150, 12, 12, YKP, TKP, 12, 0);
//   delay(300);
//   //move.CurveTo(12.497,  16.221, 30.312,  17.150, 12, 12, YKP, TKP, 12, 14, 0);
//   //delay(800);

//   //alignment
//   // back to  13.361,  17.402
//         move.facePoint(18.496,  18.198, TKP, TKI, TKD, 12, 12, true);
//         move.bto(18.496,  18.198, 12, 12, YKP, TKP, 12, 3);
//         // 15.496,  18.198
//         // was 17.361,  17.402
//         move.face(-18.895, TKP*1.2, TKI, TKD*1.2, 12, 0); 
//         // was -17.396
//         // was -12.395
//         // was h -18.935
//         delay(200);
//         leftWing.set(1);
//         //turn to H -18.395

//   // turn to 
//   /*
//       move.bto(20.196,  18.346, 8, 12, YKP, TKP, 12, 3);
//       move.face(-13.509, TKP, TKI, TKD, 12, 4); 
//       //intakePull.set(1);
//       leftWing.set(1);
//       move.voldrive(12,12);
//       move.face(-13.509, TKP, TKI, TKD, 12, 4);
//       move.to(20.196,  18.346, 8, 12, YKP, TKP, 12, 3);
//       delay(100);
//   //  move.PPSto(12.497, 16.221, 12, 12, YKP, TKP, 12, 3, 90, 3, 12);
//   //  move.face(90, TKP, TKI, TKD, 12, 4);
//   */

// ////////////////////////////////////////////////////////////

//   move.shoot = false;
//   move.cataEnabled = false;
//   cata.setVelocity(92, percent); //re enable this
//   intake.setVelocity(92,percent);
//   cata.spin(forward);
//   intake.spin(reverse);
//   delay(31200);
//   // was 28,500
//   // was 27,800
//   // was 32,800
//   intake.stop();
//   cata.stop();
//   move.cataEnabled = false;


// /////////////////////////////////////////////////////////////

// //align gate
// leftWing.set(0);
// move.facePoint(-5.137, -10.936, TKP, TKI, TKD, 12, 15, false);
// intakePull.set(0);
// move.to(-5.137, -10.936, 12, 12, YKP, TKP, 12, 9);
// // both were -5.137, -10.936 

// //intakePull.set(1);
// //intake.spin(reverse);

// //pass gate
// move.facePoint(-3.284, -85.910, TKP, TKI, TKD, 12, 15, false); // xx CHANGING TO STRAIGHT LINE
// move.to(-3.284, -85.910, 12, 12, YKP, TKP, 12, 9);
//   // move.face(180, TKP, TKI, TKD/**1.3*/, 12, 4);
//   // move.straighth(70, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 3);


// move.facePoint(6.832, -99.142, TKP, TKI, TKD, 12, 8, false);
// rightWing.set(1);
// // y was -98 and bl was 15
// move.to(6.832, -99.142, 12, 12, YKP, TKP, 12, 6);
// rightWing.set(0);

// move.face(90, TKP, TKI, TKD, 12, 4);
// leftWing.set(1);

//   move.straighth(24, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(600);
//   move.straighth(-13, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 8);
//   delay(100);
//   leftWing.set(0);
//   //intake.spin(reverse);
//   move.straighth(20, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(600);
//   move.straighth(-10, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 2);
//   //intake.spin(forward);
//   leftWing.set(0);

//   //autonStop();

// //   // back turn then to 8.228, -88.826
// //   move.facePoint(8.228, -88.826, TKP, TKI, TKD, 12, 15, true);
// //   move.bto(8.228, -88.826, 12, 12, YKP, TKP, 12, 9);

// //   // forward turn then to 24.286, -67.261 
// //   move.facePoint(26.686, -67.261 , TKP, TKI, TKD, 12, 15, false);
// //   move.to(26.686, -67.261 , 12, 12, YKP, TKP, 12, 9);

// //   // forward turn then to 34.119, -65.695 (wings out) 
// //   //move.facePoint(34.119, -65.695 , TKP, TKI, TKD, 12, 15, false);
// //   move.face(90, TKP*1.2, TKI, TKD*1.2, 12, 5); 
// //   leftWing.set(1);
// //   rightWing.set(1);
// //   delay(100);
// //   //move.to(34.119, -65.695 , 12, 12, YKP, TKP, 12, 9);
// //   move.straighth(13, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 6);
// //   leftWing.set(0);
// //   rightWing.set(0);
// //   delay(300);

// // // bcurve with 26.777, -57.800 and 50.079, -47.779  
// // //move.BCurveTo(26.777, -57.800, 50.079, -47.779, 12, 8, YKP/**1.4*/, TKP/**1.35*/, 12, 12, 5);
// //   move.facePoint(26.777, -57.800 , TKP, TKI, TKD, 12, 15, true);
// //   move.bto(26.777, -57.800, 12, 12, YKP, TKP, 12, 9);
// //   // move.facePoint(50.079, -47.779 , TKP, TKI, TKD, 12, 15, false);
// //   // move.to(50.079, -47.779 , 12, 12, YKP, TKP, 12, 9);

//   //OLD PATHING:
//    move.facePoint(20.525, -55.323, TKP, TKI, TKD, 12, 15, false);
//   move.to(20.525, -55.323, 12, 12, YKP, TKP, 12, 9);
//   // swing turn then to coord: 43.028, -51.124
//   move.swingOnRighttoPoint(44.028, -54.124, TKP, TKI, TKD*1.2, 12, 15, false);
//   move.to(44.028, -54.124, 12, 12, YKP, TKP, 12, 9);

// //move.face(95, TKP, TKI, TKD, 12, 4);
// move.face(180, TKP*1.2, TKI, TKD*1.2, 12, 10);
// // added tkp and tkd modifiers
// //intakePull.set(1);
// //intake.spin(reverse);
// move.straighth(50, 180, YKP, YKD, TKP, TKD, 12, 12, 1, 0);
// delay(1000);

// move.straighth(-25, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 6);
// delay(50);
// intakePull.set(0);
// intake.stop();
// //second
// // move.straighth(50, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
// // delay(700);

//     // back to:  44.687, -53.710
//   //move.facePoint(20.525, -53.323, TKP, TKI, TKD, 12, 15, true);
//   move.bto(44.687, -53.710, 12, 12, YKP, TKP, 12, 9);
//   //intakePull.set(1);
//   //intake.spin(reverse);
//     // forward to: 55.496, -52.911
//   move.facePoint(54.496, -54.911, TKP, TKI, TKD, 12, 15, false);
//   move.to(54.496, -54.911, 12, 12, YKP, TKP, 12, 3);
//   // both x's were 59.596

//   move.face(180, TKP*1.2, TKI, TKD*1.2, 12, 4);
//   //intakePull.set(1);
//   //intake.spin(reverse);
// move.straighth(50, 180, YKP, YKD, TKP, TKD, 12, 12, 1, 0);
// delay(1000);

// move.straighth(-40, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 9);
// //delay(500);
// intakePull.set(0);
// //intake.spin(forward);
// //intakePull.set(1);

// //DUPED
// // setup for left goal angle
//   move.facePoint(90.533, -55.145, TKP*1.2, TKI, TKD*1.2, 12, 15, false);
//   move.to(90.533, -55.145, 12, 12, YKP, TKP, 12, 0);
//   delay(600);
//     //H 219.624 
// // ram
//   //move.face(90, TKP, TKI, TKD, 12, 4);
//   move.face(180, TKP, TKI, TKD, 12, 10); 
//   move.face(230.264, TKP, TKI, TKD, 12, 4); 
//   leftWing.set(1);
//   rightWing.set(1);
//   move.straighth(30, 230.264, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(600);
//   move.face(180, TKP, TKI, TKD*1.3, 12, 4);
//   //intake.spin(reverse);
//   move.straighth(40, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(700);
//   rightWing.set(0);
//   leftWing.set(0);
//   move.straighth(-20, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 9);


// //second
// // rightWing.set(1);
// // move.straighth(50, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
// // delay(700);
// // rightWing.set(0);

//   move.facePoint(20.525, -53.323, TKP, TKI, TKD, 12, 15, true);
//   move.bto(20.525, -53.323, 12, 12, YKP, TKP, 12, 9);
//   move.face(127.616, TKP, TKI, TKD, 12, 4); 
//   leftWing.set(1);
//   rightWing.set(1);
//   move.straighth(27, 127.616, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(400);
//   move.face(180, TKP, TKI, TKD*1.3, 12, 4);
//   move.straighth(40, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(800);
//   rightWing.set(0);
//   leftWing.set(0);
//   move.straighth(-10, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 3);

//   BarHang.set(1);

  
//   // to -0.248, -75.306
//   move.facePoint(-12.248, -85.306, TKP, TKI, TKD, 12, 15, true);
//   move.bto(-12.248, -85.306, 12, 12, YKP, TKP, 12, 10);
//   move.face(0, TKP, TKI, TKD*1.3, 12, 5);
//   intakePull.set(1);
//   move.straighth(60, 0, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(900);
//   BarHang.set(0);

// autonStop();
//   /////////////////////////////

//   autonStop();
// }

// void PartialProg(){
//   auto_settings();
//   TKD = 6.548;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0); 
  
//   TKP = TKP * 1.15;

//   move.cataEnabled = false;

//   leftWing.set(1);

//   //b curve -8.728, -18.495 and -24.251, -21.154
//   move.BCurveTo(-8.7, -10.5, -24.3, -15.9, 12, 12, YKP, TKP, 12, 8, 5);
//   move.face(90, TKP, TKI, TKD, 12, 4.5);
//   move.straighth(-40, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
//   delay(600);

//   // to -12.063, -16.595
//   move.facePoint(-12.1, -18.6, TKP*1.2, TKI, TKD*1.2,  12, 4, false);
//   move.to(-12.1, -18.6, 12, 12, YKP, TKP, 12, 3.5);
//   leftWing.set(0);

//   // face -19.239
//   move.face(339, TKP*1.2, TKI, TKD*1.2, 12, 4); 

//   //move.straighth(-4, 331, YKP, YKD, TKP, TKD, 12, 12, 12, 0.5);  

//   // // b to  -9.599, -17.202 
//   // move.bto(-9.599, -17.202, 12, 12, YKP, TKP, 12, 3.5);
//   // move.face(-19.239, TKP, TKI, TKD, 12, 0);
//    rightWing.set(1);
//    move.face(339, TKP*1.2, TKI, TKD*1.2, 12, 0); 

//   /////MATCHLOAD///////////////////
//   // LeftCata.setVelocity(68, percent);
//   // RightCata.setVelocity(68, percent);
//   // LeftCata.spin(forward);
//   // RightCata.spin(forward);
//   // delay(23000); // wait for 23 seconds to shoot
//   // rightWing.set(0);

//   //delay(500);

//   ////////////////////////////////

//   // LeftCata.stop();
//   // RightCata.stop();


//   // add descore by face descore down then b curve -7.571, -13.333 and 0.760,  -7.463
//   move.facePoint(-7.6, -13.3, TKP*1.2, TKI, TKD*1.2,  12, 4, true); 
//   leftWing.set(1); 
//   rightWing.set(1);
//   move.BCurveTo(-7.6, -13.3, 1.1,  -7.5, 12, 12, YKP, TKP, 12, 12.5, 5);
//   move.swingOnRight(180, TKP, TKI, TKD, 12, 0);
//   delay(600);
//   leftWing.set(0);

//   // b curve 4.0,   5.2 and 6.6,  31.7
//   move.BCurveTo(6.0, 5.2, 6.6, 31.7, 12, 12, YKP, TKP, 12, 18, 9);
//   // x1 was 4.0

//   // b curve 5.6,  77.7 and 4.8,  85.6
//   move.BCurveTo(6.6,  77.7, 3.8,  88.6, 12, 12, YKP*0.8, TKP, 12, 17, 5);
//   // x1 was 5.6 but changed to 6.6
//   // bl was 8

//   // face then wing down then b curve 1.3,  91.1 and -6.3, 100.1 then wing up
//   move.facePoint(1.3,  91.1, TKP*1.2, TKI, TKD*1.2,  12, 8, true); 
//   leftWing.set(1);
//   move.BCurveTo(1.3,  91.1, -6.3, 100.1, 12, 12, YKP, TKP, 12, 12, 5);
//   move.swingOnRight(90, TKP, TKI, TKD, 12, 0);
//   delay(600);
//   leftWing.set(0);

// move.autondriveenabled = false;
// }

// void NoAceProg(){
//   //delay(5000);
//   auto_settings();
//    // move.facePoint(0, -24, TKP*1.3, TKI, TKD, 12, 0, true);
//   //delay(1000000);
//   TKD = 5.8;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);

//   move.cataEnabled = false;

//   // turn on later
//   intakePull.set(0);
//   intake.setVelocity(100, percent);
//   //delay(100);
//   move.cataEnabled = false;
//   TKP = TKP*1.15;

// //   //point 1:  0.961,   7.498, point 2: 9.048,  19.066
// //   move.facePoint(0.961,   7.498, TKP, TKI, TKD, 12, 15, false);
// //   move.CurveTo(0.961,   7.498, 9.048,  19.066, 12, 8, YKP/**1.4*/, TKP/**1.35*/, 12, 10, 5);

// //   //turn and go to: 24.698,  18.751
// //   move.facePoint(24.698,  18.751, TKP, TKI, TKD, 12, 15, false);
// //   move.to(24.698,  18.751, 12, 12, YKP, TKP, 12, 3);

// //   //back curve point 1: 17.826,  18.536, point 2: 6.222,   9.428
// //   move.BCurveTo(17.826,  18.536, 10.222,   9.428, 12, 8, YKP/**1.4*/, TKP/**1.35*/, 12, 12, 5);

// //   //turn to -25.530
// //         move.face(-25.530, TKP*1.2, TKI, TKD*1.2, 12, 0); 
// //         // was h -18.935
// //         delay(200);
// //         leftWing.set(1);

// //         delay(500);

// //   //MATCHLOADING
// //   leftWing.set(0);

// //   // turn then curve, point 1: -6.146,  -7.846, point 2:  -7.744, -74.448
// //   // move.facePoint(-4.146,  -7.846, TKP, TKI, TKD, 12, 7, false);
// //   // move.CurveTo(-4.146,  -7.846,  -5.744, -74.448, 12, 12, YKP/**1.4*/, TKP*0.75, 12, 26, 10);
// //   move.facePoint(-6.146,  -7.846  , TKP, TKI, TKD, 12, 15, false);
// //   move.to(-6.146,  -7.846 , 12, 12, YKP, TKP, 12, 8);
// //   move.facePoint(-5.744, -74.448  , TKP, TKI, TKD, 12, 15, false);
// //   move.to(-5.744, -74.448 , 12, 12, YKP, TKP, 12, 8);

// //   // turn curve: point 1: -7.131, -82.193, point 2: 4.335, -98.777
// //   move.facePoint(-4.131, -82.193, TKP, TKI, TKD, 12, 5, false);
// //   move.CurveTo(-4.131, -82.193,  4.335, -99.777, 12, 10, YKP/**1.4*/, TKP, 12, 12, 5);
// //   // move.facePoint(-7.131, -82.193  , TKP, TKI, TKD, 12, 15, false);
// //   // move.to(-7.131, -82.193 , 12, 12, YKP, TKP, 12, 8);
// //   // move.facePoint(4.335, -98.777  , TKP, TKI, TKD, 12, 15, false);
// //   // move.to(4.335, -98.777  , 12, 12, YKP, TKP, 12, 8);

// //   // turn then go: 22.299, -101.489  
// //   move.facePoint(22.299, -101.489  , TKP, TKI, TKD, 12, 15, false);
// //   move.to(22.299, -101.489  , 12, 12, YKP, TKP, 12, 0);
// //   delay(500);

// // autonStop();

// ////////////////////////////////////////////////////////////////////////////////////REDO AGAIN^

// // curves:  12.497,  17.221 and 25.312,  19.150 (with boosted x)
//   move.facePoint(12.497,  16.221, TKP, TKI, TKD, 12, 15, false);
//   move.to(12.497,  16.221, 12, 12, YKP, TKP, 12, 3);
//   move.facePoint(30.312,  17.150, TKP, TKI, TKD, 12, 15, false);
//   move.to(30.312,  17.150, 12, 12, YKP, TKP, 12, 0);
//   delay(300);
//   //move.CurveTo(12.497,  16.221, 30.312,  17.150, 12, 12, YKP, TKP, 12, 14, 0);
//   //delay(800);

//   //alignment
//   // back to  13.361,  17.402
//         move.bto(13.361,  17.402, 8, 12, YKP, TKP, 12, 3);
//         move.face(-15.395, TKP*1.2, TKI, TKD*1.2, 12, 0); 
//         // was h -18.935
//         delay(200);
//         leftWing.set(1);
//         //turn to H -18.395

//   // turn to 
//   /*
//       move.bto(20.196,  18.346, 8, 12, YKP, TKP, 12, 3);
//       move.face(-13.509, TKP, TKI, TKD, 12, 4); 
//       //intakePull.set(1);
//       leftWing.set(1);
//       move.voldrive(12,12);
//       move.face(-13.509, TKP, TKI, TKD, 12, 4);
//       move.to(20.196,  18.346, 8, 12, YKP, TKP, 12, 3);
//       delay(100);
//   //  move.PPSto(12.497, 16.221, 12, 12, YKP, TKP, 12, 3, 90, 3, 12);
//   //  move.face(90, TKP, TKI, TKD, 12, 4);
//   */

// ////////////////////////////////////////////////////////////

//   move.shoot = false;
//   move.cataEnabled = false;
//   cata.setVelocity(100, percent); //re enable this
//   cata.spin(forward);
//   delay(32800);
//   cata.stop();
//   move.cataEnabled = false;

// /////////////////////////////////////////////////////////////

// //align gate
// move.facePoint(-5.137, -10.936, TKP, TKI, TKD, 12, 15, false);
// leftWing.set(0);
// intakePull.set(0);
// move.to(-5.137, -10.936, 12, 12, YKP, TKP, 12, 9);

// //intakePull.set(1);
// //intake.spin(reverse);

// //pass gate
// move.facePoint(-3.284, -85.910, TKP, TKI, TKD, 12, 15, false);
// move.to(-3.284, -85.910, 12, 12, YKP, TKP, 12, 9);
// move.facePoint(6.832, -99.142, TKP, TKI, TKD, 12, 8, false);
// // y was -98 and bl was 15
// move.to(6.832, -99.142, 12, 12, YKP, TKP, 12, 6);

// move.face(90, TKP, TKI, TKD, 12, 4);
// leftWing.set(1);

//   move.straighth(24, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(600);
//   move.straighth(-13, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 8);
//   delay(100);
//   //intake.spin(reverse);
//   move.straighth(20, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(600);
//   move.straighth(-6, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 2);
//   //intake.spin(forward);
//   leftWing.set(0);

//   // turn then to coord:  20.525, -52.323
//   move.facePoint(20.525, -55.323, TKP, TKI, TKD, 12, 15, false);
//   move.to(20.525, -55.323, 12, 12, YKP, TKP, 12, 9);
//   // swing turn then to coord: 43.028, -51.124
//   move.swingOnRighttoPoint(44.028, -54.124, TKP, TKI, TKD, 12, 15, false);
//   move.to(44.028, -54.124, 12, 12, YKP, TKP, 12, 3);

// //move.face(95, TKP, TKI, TKD, 12, 4);
// move.face(180, TKP, TKI, TKD, 12, 10);
// //intakePull.set(1);
// //intake.spin(reverse);
// move.straighth(50, 180, YKP, YKD, TKP, TKD, 12, 12, 1, 0);
// delay(1000);

// move.straighth(-25, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 6);
// delay(50);
// intakePull.set(0);
// intake.stop();
// //second
// // move.straighth(50, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
// // delay(700);

//     // back to:  44.687, -53.710
//   //move.facePoint(20.525, -53.323, TKP, TKI, TKD, 12, 15, true);
//   move.bto(44.687, -53.710, 12, 12, YKP, TKP, 12, 9);
//   //intakePull.set(1);
//   //intake.spin(reverse);
//     // forward to: 55.496, -52.911
//   move.facePoint(59.496, -54.911, TKP, TKI, TKD, 12, 15, false);
//   move.to(59.496, -54.911, 12, 12, YKP, TKP, 12, 9);

//   move.face(180, TKP, TKI, TKD, 12, 4);
//   //intakePull.set(1);
//   //intake.spin(reverse);
// move.straighth(50, 180, YKP, YKD, TKP, TKD, 12, 12, 1, 0);
// delay(1000);

// move.straighth(-40, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
// delay(500);
// intakePull.set(0);
// //intake.spin(forward);
// //intakePull.set(1);

// //DUPED
// // setup for left goal angle
//   move.facePoint(90.533, -55.145, TKP, TKI, TKD, 12, 15, false);
//   move.to(90.533, -55.145, 8, 12, YKP, TKP, 12, 6);
//     //H 219.624 
// // ram
//   //move.face(90, TKP, TKI, TKD, 12, 4);
//   move.face(180, TKP, TKI, TKD, 12, 10); 
//   move.face(230.264, TKP, TKI, TKD, 12, 4); 
//   leftWing.set(1);
//   rightWing.set(1);
//   move.straighth(30, 230.264, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(600);
//   move.face(180, TKP, TKI, TKD*1.3, 12, 4);
//   //intake.spin(reverse);
//   move.straighth(40, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(700);
//   rightWing.set(0);
//   leftWing.set(0);
//   move.straighth(-20, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 9);


// //second
// // rightWing.set(1);
// // move.straighth(50, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
// // delay(700);
// // rightWing.set(0);

//   move.facePoint(20.525, -53.323, TKP, TKI, TKD, 12, 15, true);
//   move.bto(20.525, -53.323, 12, 12, YKP, TKP, 12, 9);
//   move.face(127.616, TKP, TKI, TKD, 12, 4); 
//   leftWing.set(1);
//   rightWing.set(1);
//   move.straighth(27, 127.616, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(400);
//   move.face(180, TKP, TKI, TKD*1.3, 12, 4);
//   move.straighth(40, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(800);
//   rightWing.set(0);
//   leftWing.set(0);
//   move.straighth(-10, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 3);

//   BarHang.set(1);

  
//   // to -0.248, -75.306
//   move.facePoint(-4.248, -80.306, TKP, TKI, TKD, 12, 15, true);
//   move.bto(-4.248, -80.306, 12, 12, YKP, TKP, 12, 15);
//   move.face(0, TKP, TKI, TKD*1.3, 12, 10);
//   intakePull.set(1);
//   move.straighth(50, 0, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(900);
//   BarHang.set(0);

// autonStop();
//   ////////////////////////////////////REDO AGAIN NEW STUFF^


// /*
//   //out 23.514, -52.075
//   move.facePoint(23.514, -52.075, TKP, TKI, TKD, 12, 15, false);
//   //move.to(23.514, -52.075, 8, 12, YKP, TKP, 12, 3);
//   // join curve up with 40.784, -82.843
//   move.CurveTo(23.514, -52.075, 40.784, -82.843, 12, 12, YKP, TKP, 12, 3, 12);
// */

// move.face(0, TKP, TKI, TKD, 12, 4); 
// leftWing.set(1);
// //move.straight(10, YKP, YKD, 12, 12, 3);
// move.straighth(40, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 15);
// move.swingOnRight(90, TKP, TKI, TKD, 12, 15);
// rightWing.set(1);
// move.straighth(26, 90, YKP, YKD, TKP, TKD, 12, 12, 12, 20);
// move.swingOnRight(180, TKP, TKI, TKD, 12, 0);
// delay(400);
// move.straighth(50, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
// delay(600);
// //move.straighth(-5, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 2);
// leftWing.set(0);
// move.straighth(-20, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 6);
// leftWing.set(1);
// move.straighth(30, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
// delay(500);
// leftWing.set(0);

// //back up to: 47.485, -58.244
// move.facePoint(47.485, -56.244, TKP, TKI, TKD, 12, 13, true);
// move.bto(47.485, -56.244, 12, 12, YKP, TKP, 12, 3);

// //forward to  75.445, -89.320
// move.facePoint(81.445, -56.320, TKP, TKI, TKD, 12, 10, false);
// move.to(81.445, -56.320, 12, 12, YKP, TKP, 12, 3);

// //
// move.swingOnRight(180, TKP, TKI, TKD, 12, 15);
// move.straighth(10, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 6);

// move.face(-90, TKP, TKI, TKD, 12, 8); 
// leftWing.set(1);
// rightWing.set(1);
// move.straighth(30, -90, YKP, YKD, TKP, TKD, 12, 12, 12, 21);
// leftWing.set(0);
// move.swingOnLeft(180, TKP, TKI, TKD, 12, 0);
// delay(300);
// move.straighth(50, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
// delay(600);
// rightWing.set(0);

// move.straighth(-20, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 6);
// rightWing.set(1);
// move.straighth(30, 180, YKP, YKD, TKP, TKD, 12, 12, 12, 0);
// delay(500);



// autonStop();

// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   //OLD STUFF FROM WPRA SECOND COMP (SEASON OPENER):
//   //aim H 127.616
//   move.face(127.616, TKP, TKI, TKD, 12, 4); 
//   leftWing.set(1);
//   rightWing.set(1);
//   move.straighth(27, 127.616, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(800);
//   move.face(180, TKP, TKI, TKD*1.3, 12, 4);
//   move.straighth(40, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(900);
//   rightWing.set(0);
//   leftWing.set(0);
//   move.straighth(-30, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 10);

// // setup for left goal angle
//   move.facePoint(90.533, -53.145, TKP, TKI, TKD, 12, 15, false);
//   move.to(90.533, -53.145, 8, 12, YKP, TKP, 12, 6);
//     //H 219.624 
// // ram
//   move.face(180, TKP, TKI, TKD, 12, 10); 
//   move.face(230.264, TKP, TKI, TKD, 12, 4); 
//   leftWing.set(1);
//   rightWing.set(1);
//   move.straighth(30, 230.264, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(800);
//   move.face(180, TKP, TKI, TKD*1.3, 12, 4);
//   move.straighth(40, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(700);
//   rightWing.set(0);
//   leftWing.set(0);

//   //back up to 77.043, -64.804
//   move.facePoint(77.043, -64.804, TKP, TKI, TKD, 12, 15, true);
//   move.bto(77.043, -64.804, 8, 12, YKP, TKP, 12, 3);
//   // front to 113.283, -85.368
//   move.facePoint(100.283, -85.368, TKP, TKI, TKD, 12, 15, false);
//   move.to(100.283, -85.368, 8, 12, YKP, TKP, 12, 3);
//   // front to 96.070, -101.622
//   move.facePoint(96.070, -101.622, TKP, TKI, TKD, 12, 4, false);
//   move.to(96.070, -101.622, 8, 12, YKP, TKP, 12, 3);
//   // cp ram
//   move.face(-90, TKP, TKI, TKD*1.3, 12, 4); 

//   move.straighth(24, -90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(600);
//   move.straighth(-13, -90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 4);
//   move.straighth(20, -90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(600);
//   move.straighth(-8, -90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 2);


// autonStop();
// }

// void PushSkills(){
//   delay(5000);
//   auto_settings();
//    // move.facePoint(0, -24, TKP*1.3, TKI, TKD, 12, 0, true);
//   //delay(1000000);
//   TKD = 5.8;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);

//   move.cataEnabled = false;

//   // turn on later
//   intakePull.set(0);
//   //delay(100);
//   move.cataEnabled = false;
//   TKP = TKP*1.15;


// //goal align
//   move.facePoint(16.120,  20.538, TKP, TKI, TKD, 12, 4, false);
//   move.to(16.120,  20.538, 8, 12, YKP, TKP, 12, 3);

// //drive PD press
//   move.face(90, TKP, TKI, TKD/**1.3*/, 12, 4);
//   move.straighth(-8, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 4);
//   move.straighth(20, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(650);
//   move.straighth(-8, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 5);

//   autonStop();



// ////////////////////////////////////////////////////////////
// repeat(3){
//   // was 45 shots
//   move.shoot = true;
//   delay(780);
//   // was 720 delay
// }
// /////////////////////////////////////////////////////////////


// // //load align
// //   move.facePoint(2.341,  6.704, TKP, TKI, TKD, 12, 5, false);
// //   move.to(2.341,  6.704, 12, 12, YKP, TKP, 12, 3);

// // //drive PD align
// //   //move.face(90, TKP, TKI, TKD/**1.3*/, 12, 4);
// //   move.face(180, TKP, TKI, TKD/**1.3*/, 12, 4);
// //   move.straighth(-5, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
// //   delay(500);
// //   move.face(180, TKP, TKI, TKD/**1.3*/, 12, 4);



// // gate align
// move.facePoint(-4.464,  -3.739, TKP, TKI, TKD, 12, 13, false);
// move.to(-4.464,  -3.739, 12, 12, YKP, TKP, 12, 3);

// intakePull.set(1);

// // go through gate
// leftWing.set(1);
// rightWing.set(1);
// move.facePoint(-3.284, -80.910, TKP, TKI, TKD, 12, 13, false);
// move.to(-3.284, -80.910, 6, 12, YKP, TKP, 12, 3);
// rightWing.set(0);

// //slide hold
// move.face(155, TKP, TKI, TKD/**1.3*/, 12, 4);
// move.straighth(45, 145, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 3, 12, 12, 0);
// delay(1500);
// leftWing.set(0);
// delay(50);

// //press
// move.face(90, TKP, TKI, TKD/**1.3*/, 12, 4);
// repeat(2){
// move.straighth(32, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
// delay(700);
// move.straighth(-15, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 0);
// delay(700);
// }

















// /*
//   move.facePoint(-21.146, -18.451, TKP, TKI, TKD, 12, 13, false);
//   move.to(-21.146, -18.451, 12, 8, YKP, TKP, 12, 3);
// */




// autonStop();
// }

// void swingTest(){
//     //delay(5000);
//   auto_settings();
//    // move.facePoint(0, -24, TKP*1.3, TKI, TKD, 12, 0, true);
//   //delay(1000000);
//   TKD = 5.8;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);

//   move.cataEnabled = false;
//   move.cataTarg = 142.5;

//   // turn on later
//   intakePull.set(1);

//   leftWing.set(1);
//   //delay(100);
//   move.cataEnabled = false;
//   TKP = TKP*1.15;

//   move.voldrive(12, 9);
//   move.voldrive(12, 12);

//   //move.straight(30, YKP, YKD*1.2, 12, 12, 0);
//   //delay(800);

//   //move.face(215, TKP, TKI, TKD/**1.3*/, 12, 4);

//   //leftWing.set(0);


// autonStop();
// }

// void DriverSkillsMacro(){
//     //delay(5000);
//   auto_settings();
//    // move.facePoint(0, -24, TKP*1.3, TKI, TKD, 12, 0, true);
//   //delay(1000000);
//   TKD = 5.8;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);

//   move.cataEnabled = false;
//   move.cataTarg = 142.5;

//   // turn on later
//   intakePull.set(1);
//   //delay(100);
//   move.cataEnabled = false;
//   TKP = TKP*1.15;

// // Change to Curve
//   move.facePoint(-14.549, -12.763, TKP*1.5, TKI, TKD, 12, 20, true);
//   //bl was 3
//   move.bto(-14.549, -12.763, 12, 12, YKP*1.5, TKP*1.3, 12, 3);
//   //bl was 6
//   move.face(90, TKP, TKI, TKD/**1.3*/, 12, 4);
//   move.straighth(-15, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 0);
//   delay(500);
//   move.straighth(4, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 15);

//   //move.BCurveTo(-14.549, -12.735, -30.324, -12.735, 12, 12, YKP*1.4, TKP*1.2, 12, 8, 0);
//   // x2 was -16.324 (alignment without push)
//   //delay(1200);



//   // ram 2 alliance triballs

// /* bad approach angle to matchloader
//   move.facePoint(-5.169, -8.839, TKP*1.5, TKI, TKD, 12, 25, false);
//   //bl was 3
//   move.to(-5.169, -8.839, 12, 12, YKP*1.5, TKP*1.3, 12, 3);
//   // x was -3.169
// */

//   move.facePoint(-11.430, -12.197, TKP*1.5, TKI, TKD, 12, 25, false);
//   move.to(-11.430, -12.197, 12, 12, YKP*1.5, TKP*1.3, 12, 3);

// intakePull.set(0);
//   delay(100);
// move.cataEnabled = true;
// delay(100);

//   move.face(158, TKP, TKI, TKD*1.3, 12, 4); 
//   //move.facePoint(-57.557, 81.427, TKP*1.4, TKI, TKD*1.2, 12, 2, true);
//   // x was -45

//   move.straighth(4.5, 158, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 0);
//   delay(250);

//   move.face(158, TKP, TKI, TKD*1.3, 12, 4); 

//   /*
//   move.straight(4.5, YKP, YKD*1.2, 3, 12, 0);
//   delay(250);
//   */

//   move.voldrive(0,0);


// repeat(45){
//   move.shoot = true;
//   delay(780);
//   // was 720 delay
// }




//   //MATCHLOAD (SHOULD TAKE 29 SECONDS)
//   //delay(1000);

// /*Original
//   move.facePoint(3.155, 17.548, TKP*1.5, TKI, TKD, 12, 20, true);
//   move.BCurveTo(3.155, 17.548, 4.071, 77.309, 12, 12, YKP*1.4, TKP*1.2, 12, 15, 12);
// */
//   // x1 was 5.155
//   //bl was 4
//   // h speed was 7
//   // ykp multiplier on
//   // look dist was 20

// //insert:
//   move.facePoint(1.388, 8.248, TKP*1.5, TKI, TKD, 12, 20, true);
//   move.BCurveTo(1.388, 8.248, -1.575, 73.736, 12, 12, YKP*1.4, TKP*1.35, 12, 18, 12);

//   autonStop();
// }

// void noAuto(){
//    //delay(5000);
//   auto_settings();
//    // move.facePoint(0, -24, TKP*1.3, TKI, TKD, 12, 0, true);
//   //delay(1000000);
//   TKD = 5.8;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);

//   move.cataEnabled = false;
//   move.cataTarg = 142.5;

//   // turn on later
//   intakePull.set(1);
//   //delay(100);
//   move.cataEnabled = false;
//   TKP = TKP*1.15;
//   autonStop();
//   }

// void NewProgSkills(){
//     //delay(5000);
//   auto_settings();
//    // move.facePoint(0, -24, TKP*1.3, TKI, TKD, 12, 0, true);
//   //delay(1000000);
//   TKD = 5.8;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);

//   move.cataEnabled = false;
//   move.cataTarg = 142.5;

//   // turn on later
//   intakePull.set(1);
//   //delay(100);
//   move.cataEnabled = false;
//   TKP = TKP*1.15;

// // Change to Curve
//   move.facePoint(-14.549, -12.763, TKP/**1.5*/, TKI, TKD, 12, 20, true);
//   //bl was 3
//   move.bto(-14.549, -12.763, 12, 12, YKP/**1.5*/, TKP/**1.3*/, 12, 3);
//   //bl was 6
//   move.face(90, TKP, TKI, TKD/**1.3*/, 12, 4);
//   move.straighth(-15, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(500);
//   move.straighth(4, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 15);

//   //move.BCurveTo(-14.549, -12.735, -30.324, -12.735, 12, 12, YKP*1.4, TKP*1.2, 12, 8, 0);
//   // x2 was -16.324 (alignment without push)
//   //delay(1200);



//   // ram 2 alliance triballs

// /* bad approach angle to matchloader
//   move.facePoint(-5.169, -8.839, TKP*1.5, TKI, TKD, 12, 25, false);
//   //bl was 3
//   move.to(-5.169, -8.839, 12, 12, YKP*1.5, TKP*1.3, 12, 3);
//   // x was -3.169
// */

//   move.facePoint(-11.430, -12.197, TKP/**1.5*/, TKI, TKD, 12, 25, false);
//   move.to(-11.430, -12.197, 12, 12, YKP/**1.5*/, TKP/**1.3*/, 12, 3);

// intakePull.set(0);
//   delay(100);
// move.cataEnabled = true;
// delay(100);

//   move.face(157, TKP, TKI, TKD*1.3, 12, 4); 
//   //move.facePoint(-57.557, 81.427, TKP*1.4, TKI, TKD*1.2, 12, 2, true);
//   // x was -45

//   move.straighth(4.5, 157, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 0);
//   delay(250);

//   move.face(157, TKP, TKI, TKD*1.3, 12, 4); 

//   /*
//   move.straight(4.5, YKP, YKD*1.2, 3, 12, 0);
//   delay(250);
//   */

//   move.voldrive(0,0);

// ////////////////////////////////////////////////////////////
// repeat(45){
//   move.shoot = true;
//   delay(780);
//   // was 720 delay
// }
// /////////////////////////////////////////////////////////////




//   //MATCHLOAD (SHOULD TAKE 29 SECONDS)
//   //delay(1000);

// /*Original
//   move.facePoint(3.155, 17.548, TKP*1.5, TKI, TKD, 12, 20, true);
//   move.BCurveTo(3.155, 17.548, 4.071, 77.309, 12, 12, YKP*1.4, TKP*1.2, 12, 15, 12);
// */
//   // x1 was 5.155
//   //bl was 4
//   // h speed was 7
//   // ykp multiplier on
//   // look dist was 20

// //insert:
//   move.facePoint(1.388, 8.248, TKP/**1.5*/, TKI, TKD, 12, 20, true);
//   move.BCurveTo(1.388, 8.248, -0.575, 73.736, 12, 12, YKP/**1.4*/, TKP/**1.35*/, 12, 18, 12);

//   intakePull.set(1);
//   //delay(100);

//   //crossed gate
//   //ramming right side
//   move.BCurveTo(-6.118, 88.210, -19.324, 103.513, 12, 12, YKP/**1.4*/, TKP/**1.35*/, 12, 8, 10);
//   //delay(2000);
//   //bl was 8, tkp was *1.2 mult
//   // x2 was -30 with push
//   // x2 was -16.324 (alignment without push)
//   //ensure
//   move.face(90, TKP, TKI, TKD/**1.3*/, 12, 4);
//   move.straighth(-40, 90, YKP, YKD/**1.2*/, TKP/**1.3*/, TKD*0.5, 12, 12, 12, 0);
//   delay(600);
//   move.straighth(10, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 5);
//   move.straighth(-15, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(500);
//   move.straighth(15, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 5);

//   //finish right side push


//   move.facePoint(-27.107, 54.364, TKP/**1.5*/, TKI, TKD, 12, 20, false);
//   move.to(-27.107, 54.364, 12, 12, YKP/**1.5*/, TKP/**1.3*/, 12, 20);


// /*
//   move.CurveTo(-29.445, 84.476, -31.565, 56.542, 6, 12, YKP*1.4, TKP*1.35, 12, 8, 12);
//   //bl was 5 but turn up for smooth moves
//   //move.CurveTo(-45.532, 51.354, -53.566, 70.630, 12, 12, YKP*1.4, TKP*1.35, 12, 9, 5);
//   //x1 was -54.532 and x2 was -57.566
// */

//   //move.to(-58.532, 51.354, 12, 12, YKP*1.5, TKP/**1.3*/, 12, 3); //glitch out (redo)
//   move.facePoint(-55.721, 51.293, TKP/**1.5*/, TKI, TKD, 12, 18, false);
//   move.to(-55.721, 51.293, 12, 12, YKP/**1.5*/, TKP/**1.3*/, 12, 3);

//   move.face(0, TKP, TKI, TKD/**1.3*/, 12, 4);
//   leftWing.set(1);
//   rightWing.set(1);
//   move.straighth(40, 0, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 0.5, 0);
//   delay(1800);
//   move.straighth(-14, 0, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 5);
//   leftWing.set(0);
//   rightWing.set(0);
  
//   //finish press middle

//   move.BCurveTo(-60.818, 66, -46.022, 56, 12, 12, YKP/**1.4*/, TKP/**1.35*/, 12, 8, 5);
//   //move.facePoint(-50.022, 45, TKP*1.5, TKI, TKD, 12, 10, true);
//   //move.bto(-50.022, 45, 12, 12, YKP*1.5, TKP/**1.3*/, 12, 3);

//   // move.facePoint(-60.818, 66, TKP*1.5, TKI, TKD, 12, 20, true);
//   // move.bto(-60.818, 66, 12, 12, YKP*1.5, TKP/**1.3*/, 12, 20);
//   // move.facePoint(-53.022, 56, TKP*1.5, TKI, TKD, 12, 18, true);
//   // move.bto(-53.022, 56, 12, 12, YKP*1.5, TKP/**1.3*/, 12, 3);

//   move.face(0, TKP, TKI, TKD/**1.3*/, 12, 4);
//   leftWing.set(1);
//   rightWing.set(1);
//   move.straighth(50, 0, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(900);
//   move.straighth(-14, 0, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 5);
//   leftWing.set(0);
//   rightWing.set(1);

// // angled left side
//   //move.BCurveTo(-46.705, 53.005, -96.295, 50.432, 12, 12, YKP*1.4, TKP*1.35, 12, 14, 5);
//   move.facePoint(-58.705, 49.005, TKP/**1.5*/, TKI, TKD, 12, 20, true);
//   move.bto(-58.705, 49.005, 12, 12, YKP/**1.5*/, TKP/**1.3*/, 12, 20);
//   move.facePoint(-90.295, 50.432, TKP/**1.5*/, TKI, TKD, 12, 18, true);
//   move.bto(-90.295, 50.432, 12, 12, YKP/**1.5*/, TKP/**1.3*/, 12, 3);

//   move.face(-90, TKP, TKI, TKD/**1.3*/, 12, 4);
//   move.face(180, TKP, TKI, TKD/**1.3*/, 12, 4);
//   rightWing.set(1);
//   leftWing.set(1);
//   move.face(45, TKP, TKI, TKD/**1.3*/, 12, 4);
//   move.straighth(30, 45, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(700);

//   move.face(0, TKP, TKI, TKD/**1.3*/, 12, 4);
//   move.straighth(40, 0, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(700);
//   move.straighth(-16, 0, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 4);
//   rightWing.set(0);
//   leftWing.set(1);

//   //double middle
//   // move.bto(-66.721, 46.293, 12, 12, YKP*1.5, TKP/**1.3*/, 12, 3);
//   // move.face(0, TKP, TKI, TKD/**1.3*/, 12, 4);
//   // leftWing.set(1);
//   // rightWing.set(1);
//   // move.straighth(40, 0, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   // delay(800);
//   // move.straighth(-14, 0, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 5);
//   // leftWing.set(0);
//   // rightWing.set(0);

//   //ram right
//   //autonStop();
  
//   // move.facePoint(-22.956, 62.386, TKP*1.5, TKI, TKD, 12, 12, true);
//   // move.bto(-22.956, 62.386, 12, 12, YKP*1.5, TKP/**1.3*/, 12, 3);
//   // move.facePoint(-22.056, 98.482, TKP*1.5, TKI, TKD, 12, 12, true);
//   // move.bto(-22.056, 98.482, 12, 12, YKP*1.5, TKP/**1.3*/, 12, 3);

//   move.BCurveTo(-10.118, 86.210, -21.324, 101.513, 12, 12, YKP/**1.4*/, TKP/**1.35*/, 12, 8, 10);
//   move.face(90, TKP, TKI, TKD/**1.3*/, 12, 4);
//   move.straighth(-40, 90, YKP, YKD/**1.2*/, TKP/**1.3*/, TKD*0.5, 12, 12, 12, 0);
//   delay(600);
//   move.voldrive(0,0);
//   delay(100);
//   move.straighth(10, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 5);
//   move.straighth(-15, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(500);


  
  


//   autonStop();
// }

// void FarSideAuto(){
//   //delay(5000);
// auto_settings();
//    // move.facePoint(0, -24, TKP*1.3, TKI, TKD, 12, 0, true);
//   //delay(1000000);
//   TKD = 5.8;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);

//   move.cataEnabled = false;

//   //intakePull.set(1);
//   delay(200);
//   move.cataTarg = 125;
//   move.cataEnabled = false;
//   TKP = TKP*1.15;

//   intake.setVelocity(100, percent);
//   intake.spin(reverse);

//   cata.stop();

//   //first ball    0.145,   8.353
//   move.facePoint(0.145,   8.353, TKP, TKI, TKD, 12, 7, false);
//   move.to(0.145,   8.353, 12, 12, YKP, TKP, 12, 3);

//   // move back   0.993, -16.864
//   move.facePoint(0.993, -15.864, TKP, TKI, TKD, 12, 7, true);
//   move.bto(0.993, -15.864, 12, 12, YKP, TKP, 12, 3);


//   // forward to  0.490,  -9.938
//   /*
//     move.facePoint(0.490,  -9.938, TKP, TKI, TKD, 12, 7, false);
//   move.to(0.490,  -9.938, 12, 12, YKP, TKP, 12, 3);
//   */

//   delay(300);

//   // turn 90 then forward to 16.021, -37.933
//     move.face(90, TKP, TKI, TKD/**1.3*/, 12, 20);
//     move.facePoint(16.021, -39.933, TKP, TKI, TKD, 12, 7, false);
//     rightWing.set(1);
//   move.to(16.021, -39.933, 12, 12, YKP, TKP, 12, 3);
//   rightWing.set(0);

//   // pure ram at 90 with outtake then intake up
//   move.face(90, TKP, TKI, TKD/**1.3*/, 12, 4);
//   intake.spin(forward);
//   intakePull.set(0);
//   delay(300);
//   move.straighth(20, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 0);
//   delay(600);
//   move.straighth(-15, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 4);

//   // forward to while intaking 25.960, -14.247 intake down spin forward
//       move.facePoint(25.960, -16.247, TKP, TKI, TKD, 12, 7, false);
//       intake.spin(reverse);
//       //intakePull.set(1);
//   move.to(25.960, -14.247, 12, 12, YKP, TKP, 12, 3);

//   // second ball 54.225,   4.804 
//         move.facePoint(54.225,   5.304 , TKP, TKI, TKD, 12, 7, false);
//   move.to(54.225,   5.304 , 12, 12, YKP, TKP, 12, 3);
//   delay(300);
//   // face 90 then pure ram at 180
//   move.face(90, TKP, TKI, TKD/**1.3*/, 12, 20);
//   move.face(180, TKP, TKI, TKD/**1.3*/, 12, 3);
//   leftWing.set(1);
//   intake.spin(forward);
//   move.straighth(60, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(1000);
//   leftWing.set(0);
//   move.straighth(-15, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 4);

//   // third ball 43.337,   4.423
//   intake.spin(reverse);
//            move.facePoint(41.337,   5.723 , TKP, TKI, TKD, 12, 7, false);
//   move.to(41.337,   5.723 , 12, 12, YKP, TKP, 12, 3);
//   // back up to  55.592,  -5.384
//              move.facePoint(55.592,  -5.384 , TKP, TKI, TKD, 12, 7, true);
//   move.bto(55.592,  -5.384 , 12, 12, YKP, TKP, 12, 3);


//   move.face(180, TKP, TKI, TKD/**1.3*/, 12, 3);
//   intake.spin(forward);
//   move.straighth(60, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(1000);
//   move.straighth(-15, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 4);



//   autonStop();
//   move.facePoint(18.699,  19.042, TKP, TKI, TKD, 12, 15, false);
//   move.to(18.699,  19.042, 8, 12, YKP, TKP, 12, 3);



//   autonStop();
//   //////////////////////////////////CUTOUT/////////////////////////////////////////////////////////////////////////
//     //delay(5000);
//   auto_settings();
//    // move.facePoint(0, -24, TKP*1.3, TKI, TKD, 12, 0, true);
//   //delay(1000000);
//   TKD = 5.8;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);

//   move.cataEnabled = false;

//   // turn on later
//   intakePull.set(0);
//   delay(200);
//   move.cataTarg = 125;
//   move.cataEnabled = true;
//   TKP = TKP*1.15;

//   intake.setVelocity(100, percent);
//   intake.spin(forward);

//   delay(400);

//   move.to(-0.077, 9.625, 12, 12, YKP*1.5, TKP*1.3, 12, 8);

//   move.facePoint(0.382, -12.317, TKP*1.5, TKI, TKD, 12, 20, true);
//   move.bto(0.382, -12.317, 12, 12, YKP*1.5, TKP*1.3, 8, 15);
//   move.facePoint(16.167, -39.902, TKP*1.5, TKI, TKD, 12, 20, true);
//   move.bto(16.167, -39.902, 12, 12, YKP*1.5, TKP*1.3, 8, 3);

//   move.face(-90, TKP, TKI, TKD/**1.3*/, 12, 25);
//   move.straighth(-25, -90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 0);
//   delay(650);
//   move.straighth(10, -90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 8);

//   move.face(0, TKP, TKI, TKD/**1.3*/, 12, 25);
//   move.face(90, TKP, TKI, TKD/**1.3*/, 12, 4);
//   intake.spin(reverse);
//   delay(200);
//   move.straighth(15, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 0);
//   delay(650);
//   move.straighth(-10, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 5);

//   intake.spin(forward);

//   move.facePoint(25.458, -15.141, TKP*1.5, TKI, TKD, 12, 6, false);
//   move.to(25.458, -15.141, 12, 12, YKP*1.5, TKP*1.3, 8, 5);
//   move.facePoint(50.636, 7.395, TKP*1.5, TKI, TKD, 12, 6, false);
//   move.to(50.636, 7.395, 12, 12, YKP*1.5, TKP*1.3, 8, 5);

//   move.face(90, TKP, TKI, TKD/**1.3*/, 12, 4);
//   move.face(180, TKP, TKI, TKD/**1.3*/, 12, 4);

//   leftWing.set(1);

//   intake.spin(reverse);

//   move.straighth(50, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(750);
//   move.straighth(-10, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 5);  
//   leftWing.set(0);

//   intake.spin(forward);
//   move.facePoint(38.349, 5.013, TKP, TKI, TKD, 12, 6, false);
//   move.to(38.349, 5.013, 12, 12, YKP*1.5, TKP*1.3, 12, 5);

//   move.facePoint(50.636, 7.395, TKP*1.5, TKI, TKD, 12, 6, true);
//   move.bto(50.636, 7.395, 12, 12, YKP*1.5, TKP*1.3, 12, 5);

//   move.face(180, TKP, TKI, TKD/**1.3*/, 12, 4);
//   intake.spin(reverse);
//   delay(200);
//   move.straighth(75, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(950);
//   move.straighth(-10, 180, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 5);







//   autonStop();

// }

// void CloseSideAuto(){
//   //delay(5000);
//   auto_settings();
//    // move.facePoint(0, -24, TKP*1.3, TKI, TKD, 12, 0, true);
//   //delay(1000000);
//   TKD = 5.8;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);

//   move.cataEnabled = false;

//   // turn on later
//   intakePull.set(0);
//   move.cataTarg = 125;
//   move.cataEnabled = false;
//   TKP = TKP*1.15;

//   // back to  -7.195, -15.676
//   move.facePoint(-7.195, -15.676, TKP, TKI, TKD, 12, 7, true);
//   move.bto(-7.195, -15.676, 6, 12, YKP, TKP, 12, 3);

//   // turn to 46.921
//   move.face(46.921, TKP, TKI, TKD, 12, 4);

//   // wing out 
//   rightWing.set(1);

//   // to -3.455, -12.540
//   move.to(-3.455, -12.540, 12, 12, YKP, TKP, 12, 3);

//   // turn to 50
//   move.face(0, TKP, TKI, TKD, 12, 4);

//   rightWing.set(0);

//   move.facePoint(5.796, 6.533, TKP, TKI, TKD, 12, 7, false);
//   move.to(5.796, 6.533, 6, 12, YKP, TKP, 12, 3);

//   move.face(0, TKP, TKI, TKD/**1.3*/, 12, 4);
//   move.straighth(24, 0, YKP, YKD, TKP, TKD, 12, 12, 12, 3);

//   autonStop();



//   intake.setVelocity(100, percent);
//   //intake.spin(forward);

// // forward to 6.980,   8.51
//   move.facePoint(6.980,   8.51, TKP, TKI, TKD, 12, 7, false);
//   move.to(6.980,   8.51, 6, 12, YKP, TKP, 12, 3);

// //back to  1.075,   8.845
//   move.facePoint( 1.075,   9.245, TKP, TKI, TKD, 12, 7, true);
//   move.bto( 1.075,   9.245, 6, 12, YKP, TKP, 12, 3);

// poleGrab.set(1);

// move.face(0, TKP, TKI, TKD/**1.3*/, 6, 4);

// delay(200);

// poleGrab.set(0);

// //forward to  26.797, -14.977 
//   move.facePoint(26.797, -14.977, TKP, TKI, TKD, 12, 7, false);
//   move.to(26.797, -14.977, 6, 12, YKP, TKP, 12, 3);
// // intake drop and cata down 
// intakePull.set(1);
// delay(300);
// move.cataEnabled = false;
// // forward to 18.336, -29.061
//   move.facePoint(18.336, -29.061, TKP, TKI, TKD, 12, 7, false);
//   move.to(18.336, -29.061, 6, 12, YKP, TKP, 12, 3);
// // turn to h -154.266
//   move.face(-154.266, TKP, TKI, TKD/**1.3*/, 12, 4);


// autonStop();
//   move.facePoint(8.625,  17.870, TKP, TKI, TKD, 12, 7, false);
//   move.to(8.625,  17.870, 6, 12, YKP, TKP, 12, 3);

//   autonStop();



//   ///////////////////////////////CUTOUT/////////////////////////////////////

//   move.facePoint(-14.549, -12.763, TKP*1.5, TKI, TKD, 12, 20, true);
//   //bl was 3
//   move.bto(-14.549, -12.763, 12, 12, YKP*1.5, TKP*1.3, 12, 3);
//   //bl was 6
//   move.face(90, TKP, TKI, TKD/**1.3*/, 12, 4);
//   move.straighth(-15, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 0);
//   delay(500);
//   move.straighth(4, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 15);

//   move.facePoint(-7.974, -14.536, TKP*1.5, TKI, TKD, 12, 6, false);
//   move.to(-7.974, -14.536, 12, 12, YKP*1.5, TKP*1.3, 12, 5);

//   intake.spin(forward);

//   move.face(135, TKP, TKI, TKD/**1.3*/, 12, 4);
//   move.straighth(7, 135, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 0);
//   delay(2000);
//   move.straighth(-10, 135, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 5);

//   move.facePoint(2.606, 4.063, TKP*1.5, TKI, TKD, 12, 6, false);
//   move.to(2.606, 4.063, 12, 12, YKP*1.5, TKP*1.3, 12, 5);


//   move.face(-10, TKP, TKI, TKD/**1.3*/, 12, 25);
//   move.face(0, TKP, TKI, TKD/**1.3*/, 12, 4);
//   intake.spin(reverse);
//   intakePull.set(1);
//   move.straighth(35, 0, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 4);






//   autonStop();
// }



// void DoubleLoadSkills(){
//     //delay(5000);
//   auto_settings();
//    // move.facePoint(0, -24, TKP*1.3, TKI, TKD, 12, 0, true);
//   //delay(1000000);
//   TKD = 5.8;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);

//   move.cataEnabled = false;

//   // turn on later
//   intakePull.set(1);
//   //delay(100);
//   move.cataEnabled = false;
//   TKP = TKP*1.15;

// // Change to Curve
//   move.facePoint(-14.549, -12.763, TKP*1.5, TKI, TKD, 12, 20, true);
//   //bl was 3
//   move.bto(-14.549, -12.763, 12, 12, YKP*1.5, TKP*1.3, 12, 3);
//   //bl was 6
//   move.face(90, TKP, TKI, TKD/**1.3*/, 12, 4);
//   move.straighth(-15, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 0);
//   delay(500);
//   move.straighth(4, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 15);

//   //move.BCurveTo(-14.549, -12.735, -30.324, -12.735, 12, 12, YKP*1.4, TKP*1.2, 12, 8, 0);
//   // x2 was -16.324 (alignment without push)
//   //delay(1200);



//   // ram 2 alliance triballs

// /* bad approach angle to matchloader
//   move.facePoint(-5.169, -8.839, TKP*1.5, TKI, TKD, 12, 25, false);
//   //bl was 3
//   move.to(-5.169, -8.839, 12, 12, YKP*1.5, TKP*1.3, 12, 3);
//   // x was -3.169
// */

//   move.facePoint(-11.430, -12.197, TKP*1.5, TKI, TKD, 12, 25, false);
//   move.to(-11.430, -12.197, 12, 12, YKP*1.5, TKP*1.3, 12, 3);

// intakePull.set(0);
//   delay(100);
// move.cataEnabled = true;
// delay(100);

//   move.face(158, TKP, TKI, TKD*1.3, 12, 4); 
//   //move.facePoint(-57.557, 81.427, TKP*1.4, TKI, TKD*1.2, 12, 2, true);
//   // x was -45

//   move.straighth(4.5, 158, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 0);
//   delay(250);

//   move.face(158, TKP, TKI, TKD*1.3, 12, 4); 

//   /*
//   move.straight(4.5, YKP, YKD*1.2, 3, 12, 0);
//   delay(250);
//   */

//   move.voldrive(0,0);

// /*
// repeat(23){
//   move.shoot = true;
//   delay(720);
//   // was 720 delay
// }
// */

//   move.facePoint(0.388, 8.248, TKP*1.5, TKI, TKD, 12, 20, true);
//   move.BCurveTo(0.388, 8.248, -1.575, 73.736, 12, 12, YKP*1.4, TKP*1.35, 12, 18, 12);
//   intakePull.set(1);
//   move.BCurveTo(-19.081, 77.054, -19.818, 59.054, 12, 12, YKP*1.4, TKP*1.35, 12, 13, 12);
//   move.to(-51.481, 54.673, 12, 12, YKP*1.5, TKP*1.3, 12, 3);

//   move.face(0, TKP, TKI, TKD/**1.3*/, 12, 4);
//   leftWing.set(1);
//   rightWing.set(1);
//   move.straighth(40, 0, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 0);
//   delay(800);
//   move.straighth(-14, 0, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 12, 12, 12, 2);
//   leftWing.set(0);
//   rightWing.set(0);








// autonStop();
// }

void offsetFinder(){
  delay(5000);
  auto_settings();
   // move.facePoint(0, -24, TKP*1.3, TKI, TKD, 12, 0, true);
  //delay(1000000);
  TKD = 5.4;
  starttime = Brain.timer(msec);

  intake_lift = false;
  chassis.setPos(0, 0, 0);

  move.cataEnabled = false;

  // intakePull.set(1);

repeat(10){
  move.face(90, TKP, TKI, TKD, 8, 4);
  move.face(180, TKP, TKI, TKD, 8, 4);
  move.face(270, TKP, TKI, TKD, 8, 4);
  move.face(360, TKP, TKI, TKD, 8, 2);
}

autonStop();
}

// void ProgSkills(){
//   //delay(5000);
//   auto_settings();
//    // move.facePoint(0, -24, TKP*1.3, TKI, TKD, 12, 0, true);
//   //delay(1000000);
//   TKD = 5.8;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);

//   move.cataEnabled = false;

//   // turn on later
//   intakePull.set(0);
//   delay(100);
//   move.cataEnabled = true;
//   TKP = TKP*1.15;

//   move.facePoint(-12.997, -18.609, TKP, TKI, TKD, 12, 4, true);
//   move.bto(-12.997, -18.609, 12, 12, YKP, TKP * 1.3, 12, 8);

//   move.face(90, TKP, TKI, TKD/**1.3*/, 12, 4);
//   move.straighth(-15, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 0);
//   delay(500);
//   move.straighth(4, 90, YKP, YKD*1.2, TKP*1.3, TKD*0.5, 6, 12, 12, 2);

//   move.face(61.12, TKP, TKI, TKD/**1.3*/, 12, 3);
//   move.to(-5.250, -10.664, 12, 12, YKP, TKP*1.3, 12, 3);
//   move.face(155.85, TKP, TKI, TKD/**1.3*/, 12, 3);
//   move.straighth(4, 155.85, YKP, YKD*1.2, TKP*1.3, TKD, 12, 12, 12, 0);
//   delay(500);


// // MATCHLOAD HERE

//   delay(1000);
//   move.face(200, TKP, TKI, TKD/**1.3*/, 12, 3);
//   // was 219.81

//   // NEW
//   //move.facePoint(-4.472, -5.381, TKP, TKI, TKD, 12, 3, true);

//   // Fusing this into one curve
//   // move.bto(-6.238, -6.460, 12, 12, YKP, TKP * 1.3, 12, 3);

//   // move.face(178, TKP, TKI, TKD/**1.3*/, 12, 2);
//   // move.straighth(-72, 178, YKP, YKD*1.2, TKP*1.3, TKD, 12, 12, 12, 1);

// /*
//   //OLD
//   move.BCurveTo(-6.238, 0.460, -11.238, 65.762, 12, 12, YKP, TKP, 12, 28, 4);
//   // y speed was 6v h speed was 10v
// */

//   //NEW
//   move.BCurveTo(-7.972, 0, -9.373, 64.789, 12, 7, YKP, TKP*1.2, 12, 20, 4);
//   // coord 2 x was -9.373
//   // coord 1 y was 5
//   //look distance was 28 inches



//   // Just Crossed Gateway

//   intakePull.set(1);


//   autonStop();
// }

// void test(){
//   //delay(5000);
//   auto_settings();
//    // move.facePoint(0, -24, TKP*1.3, TKI, TKD, 12, 0, true);
//   //delay(1000000);
//   TKD = 5.8;
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   chassis.setPos(0, 0, 0);
//   move.cataEnabled = true;
//   TKP = TKP*1.15;

//   move.facePoint(-9.105, -17.385, TKP, TKI, TKD, 12, 4, false);
//   //move.straighth(-9.105, -17.385, YKP, YKD*.8, TKP*1.3, TKD, 6, 12, 12, 0);
//   move.to(-9.105, -17.385, 6, 12, YKP, TKP * 1.3, .8, 2);

//   move.facePoint(3.609, -25.713, TKP, TKI, TKD, 8, 2, false);
//   //move.to(-3.609, -25.713, 6, 12, YKP, TKP * 1.3, .8, 2);
//   move.straighth(9, -194.83, YKP, YKD*1.2, TKP*1.3, TKD, 12, 12, 12, 1);

//   delay(1000);

//   move.face(-270, TKP, TKI, TKD*1.3, 12, 5);
//   move.straighth(-20, -270, YKP, YKD*1.2, TKP*1.3, TKD, 8, 12, 12, 1);

//   move.facePoint(10.641, -5.096, TKP, TKI, TKD, 12, 4, false);
//   move.to(10.641, -5.096, 12, 12, YKP, TKP * 1.3, .8, 2);
//   // yspeed was 6 volts

//   move.face(-180, TKP, TKI, TKD*1.3, 12, 1);
//   move.straighth(-70, -180, YKP, YKD*1.2, TKP*1.3, TKD, 12, 12, 12, 2);
  

//   //just crossed field ^
//   /*
//   move.facePoint(-10.740, 62.147, TKP, TKI, TKD, 12, 4, false);
//   move.CurveTo(-10.470, 62.147, -10.339, 30.233, 6, 6, YKP, TKP*1.3, 12, 12, 2);

//   autonStop();

//   move.facePoint(-14.339, 30.233, TKP, TKI, TKD, 12, 4, false);
//   move.to(-14.339, 30.233, 12, 12, YKP, TKP * 1.3, .8, 2);

//   move.facePoint(-53.215, 37.735, TKP, TKI, TKD, 12, 4, false);
//   move.to(-53.215, 37.735, 12, 12, YKP, TKP * 1.3, .8, 2);
//   */


  



// autonStop();
// }