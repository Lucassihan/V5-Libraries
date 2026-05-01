// OLD MECHA SKILLS (NOT DONE):
//void MechaSkills(){
//   auto_settings();
//   enableLoadLB = false;
//   enableMidLB = false;
//   enableScoreLB = false;
//   enableLowerLB = false;
//   delay(7000);
//   starttime = Brain.timer(msec);

//   intake_lift = false;
//   // chassis.setPos(0, -6.8, 0); // jig start, 13 hole c channel back from the easy setup (6.8 inches back)
//   chassis.setPos(0, 0, 0);

//   // // GOAL 1 ////////////////////////////////////////////////
//   intake.setVelocity(100, percent);
//   intake.spin(forward);
//   delay(500);
//   intake.spin(reverse);
//   delay(100);
//   // // move to 0.2, 5.5
//   move.to(0.2, 5.5, 8, 12, YKP, TKP, 12, 3);
//   intake.stop();
//   //delay(200);

//   // face b to 21.8, 9.5
//   move.facePoint(21.8, 8.5, TKP*0.7, TKI, TKD*1.2, 7, 7, true);
//   //delay(400);
//   move.bto(21.8, 8.5, 4, 12, YKP, TKP, 12, 3);
//   BackMech.set(1);
//   //delay(500);
//   intake.setVelocity(100, percent);
//   intake.spin(forward);

//   move.facePoint(22.0, 27.1, TKP*0.7, TKI, TKD*1.35, 12, 11, false);
//   delay(100);
//   move.to(22.0, 27.1, 10, 12, YKP, TKP, 12, 2);

//   move.facePoint(34.8, 39.0, TKP, TKI, TKD*1.2, 12, 11, false);
//   delay(100);
//   move.to(34.8, 39.0, 12, 12, YKP, TKP, 12, 9);
//   delay(100); // was 300

//   enableLoadLB = true;
//   autonloadLBCommand();
//   move.facePoint(46.1, 78.0, TKP*1.4, TKI, TKD, 12, 5, false);
//   delay(100);
//   move.to(46.1, 78.0, 8, 12, YKP, TKP, 12, 5);
//   delay(100); // settle + load

//   move.facePoint(39.9, 54.7, TKP*1.3, TKI, TKD, 12, 4, true);
//   move.bto(39.9, 54.7, 10, 12, YKP, TKP, 12, 4);
//   delay(100);
//   enableLoadLB = false;
//   intake.spinFor(-60, degrees, true);
//   delay(50);
//   //delay(200);
//   enableMidLB = true;
//   autonmidLBCommand();
//   delay(100);

//   intake.setVelocity(100, percent);
//   intake.spin(forward);
//   move.facePoint(55.7, 55.2, TKP*0.7, TKI, TKD*1.2, 9, 4, false);
//   move.to(55.7, 55.2, 12, 12, YKP, TKP, 12, 3);
//   move.face(100, TKP, TKI, TKD, 12, 8);
//   move.voldrive(3, 3);
//   delay(400);
//   move.straighth(-3, chassis.h, YKP, YKD, TKP, TKD, 12, 12, 12, 2);
//   enableMidLB = false;
//   enableScoreLB = true;
//   autonscoreLBCommand();
//   delay(400);
//   //delay(100);
//   move.voldrive(-2, 2);
//   delay(50);

//   //move.straighth(-10, chassis.h, YKP, YKD, TKP, TKD, 12, 12, 12, 2);
//   enableScoreLB = false;
//   enableLowerLB = true;
//   autonlowerLBCommand();
//   delay(300);
//   move.facePoint(43.6, 55.7, TKP, TKI, TKD*1.3, 12, 2, true);
//   move.bto(43.6, 55.7, 8.5, 12, YKP*1.15, TKP, 12, 2);
//   intake.setVelocity(100, percent);
//   intake.spin(forward);
//   move.face(90, TKP, TKI, TKD, 12, 8);
//   move.face(180, TKP, TKI, TKD*1.3, 12, 4);
//   move.straighth(58, 180, YKP, YKD, TKP, TKD, 7, 12, 12, 2); // speed was 5
//   move.swingOnRighttoPoint(54.3, 8.6, TKP, TKI, TKD, 5, 5, false);
//   move.straighth(13, chassis.h, YKP, YKD, TKP, TKD, 5, 12, 12, 2); // speed was 5
//   delay(600);
//   //move.facePoint(60.3, -1.0, TKP, TKI, TKD, 12, 7, true);
//   move.swingOnRighttoPoint(56.5, -1.5, TKP, TKI, TKD*1.2, 7, 10, true);
//   move.bto(56.5, -1.5, 4, 12, YKP*1.3, TKP, 12, 0);
//   delay(900);
//   intake.spin(reverse);
//   delay(100);
//   BackMech.set(0);
//   delay(200);
//   intake.stop();
//   move.voldrive(3, 3);
//   delay(250);

//   // SECOND GOAL ///////////////////////////////////////////////////////////////////////////////////////
//   move.swingOnLefttoPoint(-3.1, 3.75, TKP*1.2, TKI, TKD, 8, 4, false); // was TKD boost 1.2 TKP boost 0
//   // move.facePoint(-3.1, 3.5, TKP, TKI, TKD*1.2, 7, 4, false);
//   //delay(200);
//   move.to(-3.1, 3.75, 7, 12, YKP*1.3, TKP, 12, 5);
//   delay(400);
//   // delay(400);
//   // move.face(0, TKP, TKI, TKD, 7, 8);
//   // move.face(90, TKP, TKI, TKD, 7, 2);
//   // move.straighth(-20, 90, YKP, YKD, TKP, TKD, 4, 12, 12, 2);
//   move.facePoint(-27.7, 3.5, TKP, TKI, TKD*1.3, 7, 3, true);
//   delay(400);
//   move.bto(-27.7, 3.5, 4, 12, YKP, TKP, 12, 3);
//   BackMech.set(1);
//   //delay(200);
//   intake.setVelocity(100, percent);
//   intake.spin(forward);

//   move.facePoint(-28.0, 18.8, TKP*0.7, TKI, TKD*1.2, 9, 11, false);
//   delay(100);
//   // autonStop();
//   // move.to(-29.1, 26.7, 10, 12, YKP, TKP, 12, 2);
//   move.straighthToPoint(16, -29.1, 26.7, YKP, YKD, TKP, TKD, 12, 12, 12, 3, false);
//   move.facePoint(-45.5, 44.5, TKP*1.2, TKI, TKD*0.7, 12, 11, false);
//   delay(100);
//   move.to(-45.5, 44.5, 12, 12, YKP, TKP, 12, 12);
//   delay(100); // was 300

//   move.facePoint(-50.0, 64.1, TKP, TKI, TKD, 9, 5, false);
//   delay(100);
//   move.to(-50.0, 64.1, 8, 12, YKP, TKP, 12, 5);
//   enableLowerLB = false;
//   enableLoadLB = true;
//   autonloadLBCommand();
//   move.voldrive(8, 8);
//   delay(200); // settle + load

//   move.facePoint(-47.0, 52.0, TKP, TKI, TKD, 12, 4, true);
//   move.bto(-47.0, 52.0, 10, 12, YKP, TKP, 12, 4);
//   delay(100);
//   enableLoadLB = false;
//   intake.spinFor(-60, degrees, true);
//   delay(50);
//   //delay(200);
//   enableMidLB = true;
//   autonmidLBCommand();
//   delay(100);

//   intake.setVelocity(100, percent);
//   intake.spin(forward);
//   move.facePoint(-61.1, 52.0, TKP*0.7, TKI, TKD*1.2, 9, 4, false);
//   move.to(-61.1, 52.0, 12, 12, YKP, TKP, 12, 3);
//   move.face(270, TKP, TKI, TKD, 12, 8);
//   move.voldrive(4, 4);
//   delay(700);
//   move.straighth(-3, chassis.h, YKP, YKD, TKP, TKD, 12, 12, 12, 2);
//   enableMidLB = false;
//   enableScoreLB = true;
//   autonscoreLBCommand();
//   delay(400);
//   enableScoreLB = false;
//   enableLowerLB = true;
//   autonlowerLBCommand();
//   delay(200);
//   move.voldrive(-2, -2);
//   delay(50);

//   //move.straighth(-10, chassis.h, YKP, YKD, TKP, TKD, 12, 12, 12, 2);
//   move.facePoint(-53.8, 51.8, TKP, TKI, TKD, 12, 2, true);
//   move.bto(-53.8, 51.8, 9, 12, YKP*1.15, TKP, 12, 2);
//   intake.setVelocity(100, percent);
//   intake.spin(forward);
//   move.face(270, TKP, TKI, TKD, 12, 8);
//   move.face(180, TKP, TKI, TKD, 12, 2);
//   move.straighth(58, 180, YKP, YKD, TKP, TKD, 7, 12, 12, 2); // speed was 5
//   move.swingOnLefttoPoint(-63.1, 4.3, TKP, TKI, TKD, 5, 5, false);
//   move.straighth(13, chassis.h, YKP, YKD, TKP, TKD, 5, 12, 12, 2); // speed was 5
//   delay(600);
//   move.swingOnLefttoPoint(-64.2, -10.5, TKP, TKI, TKD*1.1, 7, 7, true);
//   move.bto(-64.2, -10.5, 4, 12, YKP*1.3, TKP, 12, 0);
//   delay(900);
//   intake.spin(reverse);
//   delay(100);
//   BackMech.set(0);
//   delay(200);
//   intake.stop();
//   move.voldrive(3, 3);
//   delay(250);



//   autonStop();

// }

// MOA AND LOCAL COMPS UNTIL BEFORE MECHA:
  // OLD CHASSIS OFFSETS:
  //   //chassis.SS = 7.835213764, chassis.SR =  -2.479220211; // big old offsets
  //  // chassis.SS = -0.206901426, chassis.SR = -2.636178815; // small old offsets

  //  //chassis.SS = 5.869634301396866, chassis.SR = -1.788583337909;
  //  //chassis.SS = 4.8098215406, chassis.SR = -1.81420728499;

  //  //chassis.SS = 4.4098215406, chassis.SR = -1.8013953114495;
  //  // ss sideways, sr forwards

  //  //chassis.SS = 6.439649681528662, chassis.SR = -1.129299363057325;

  // //chassis.SS = 6.115764331210191, chassis.SR = -1.8013953114495;
  // // chassis.SS = 5.27, chassis.SR = -1.79; // SR was negative
  // // chassis.SS = 9.143, chassis.SR = -0.558;

  // //chassis.SS = 0.00000000000001, chassis.SR = -0.000000000000000001; // for match autos
  // chassis.SS = -2.03925228584, chassis.SR = 0.82092119646; // for skills

// void simpleSkills(){
//   delay(7000);
//   Stick.set(0);
//   NeutralLock.set(0);
//   auto_settings();
//   starttime = Brain.timer(msec);
//   intake_lift = false;
//   chassis.setPos(0, 0, 0);
//   //test this
//   TKP = 0.389;
//   TKI = 0.012;
//   TKD = 2.715;
//   YKP = 1.0; // was 1.5
//   YKI = 0;
//   YKD = 8.5;

//   move.cataEnabled = false;
//   enableNeutralStopper = false;
//   enableStopper = false;

//   intake.setVelocity(100, percent);
//   move.straighth(-10, chassis.h, YKP, YKD, TKP, TKD, 2, 12, 12, 0);
//   delay(1000);
//   intake.spin(forward);
//   delay(700);
//   intake.stop();

//     // to 0.1, 8.9
//   move.to(0.1, 8.9, 7, 12, YKP, TKP, 12, 6);
//   delay(200);

//   // b face to (to mogo) 27.1, 8.5 
//   move.facePoint(30.1, 8.5, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(30.1, 8.5, 4, 12, YKP, TKP, 12, 9);
//   //delay(200);

//   BackMech.set(1);
//   move.voldrive(0, 0);
//   delay(400); // no swing

//   intake.setVelocity(100, percent);
//   intake.spin(forward);

//   // face to 32.8, 36.4
//   move.facePoint(32.8, 36.4, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(32.8, 36.4, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to 48.3, 55.4
//   move.facePoint(48.3, 55.4, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(48.3, 55.4, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to 81.4, 73.0
//   move.facePoint(81.4, 73.0, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(81.4, 73.0, 7, 12, YKP, TKP, 12, 6);
//   //delay(800);

//   // b to (no face) 68.5, 65.9
//   move.facePoint(68.5, 65.9, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(68.5, 65.9, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to 66.1, 43.2
//   move.facePoint(66.1, 43.2, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(66.1, 43.2, 7, 12, YKP, TKP, 12, 11); // bl was 6
//   //delay(200);

//   // face to 66.1, 13.9
//   move.facePoint(66.1, 13.9, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(66.1, 13.9, 7, 12, YKP, TKP, 12, 11); // bl was 6
//   //delay(200);

//   // face to 66.2, -4.7
//   move.facePoint(66.2, -4.7, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(66.2, -4.7, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // b to (no face) 68.4, 35.2
//   move.facePoint(68.4, 35.2, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(68.4, 35.2, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to 80.5, 11.5 
//   move.facePoint(80.5, 11.5, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(80.5, 11.5, 7, 12, YKP, TKP, 12, 6);
//   //delay(400);

//   // b to 56.2, 23.2
//   move.bto(56.2, 23.2, 7, 12, YKP, TKP*0.8, 7, 6);

//   // b face to 76.4, -6.3
//   move.facePoint(76.4, -6.3, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(76.4, -6.3, 6, 12, YKP, TKP, 7, 9);
//   //delay(200);

//   // mogo drop off
//   move.face(315, TKP, TKI, TKD, 12, 8);
//   BackMech.set(0);
//   delay(200);
//   intake.stop();

//   // SECOND MOGO /////////////////////////////////////////////////////////
//   //move.PPSbto(-33.8, 3.9, 6, 6, YKP, TKP/**0.6*/, 12, 3/*BL*/, 40/*RAng*/, 6/*TE*/, 22/*Look*/);
//     // to (no face) 63.8, 6.4
//   move.to(63.8, 5.2, 7, 12, YKP, TKP, 12, 6);
//   delay(200);

//   // face to -6.0, 5.2
//   // was 8.4 on y
//   move.facePoint(-6.0, 5.2, TKP*1.2, TKI, TKD*1.3, 7, 6, false);
//   delay(200);
//   move.to(-6.0, 5.2, 12, 12, YKP, TKP*1.2, 12, 6);
//   delay(200);

//   // face b to -37.6, 6.3
//   // was 8.4 on y
//   move.facePoint(-37.6, 5.2, TKP, TKI, TKD*1.3, 7, 6, true);
//   delay(200);
//   //chassis.setPos(-2.0, 3.9, chassis.h); // RESET (x is offset to the right to push entire route to the left)
//   move.bto(-37.6, 5.2, 4, 12, YKP, TKP, 12, 7);
//   //delay(200);

//   // clamp
//   BackMech.set(1);
//   move.voldrive(0, 0);
//   delay(400); // no swing
//   intake.setVelocity(100, percent);
//   intake.spin(forward);

//   // face to -35.4, 34.0
//   move.facePoint(-35.4, 34.0, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(-35.4, 34.0, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to -63.7, 37.3
//   move.facePoint(-63.7, 33.3, TKP*1.2, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(-63.7, 33.3, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to -81.7, 69.7
//   move.facePoint(-81.7, 69.7, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(-81.7, 69.7, 7, 12, YKP, TKP, 12, 6);
//   //delay(400);

//   // b to (no face) -64.5, 45.9 
//   move.facePoint(-64.5, 45.9, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(-64.5, 45.9, 7, 12, YKP, TKP, 12, 6);

//   // face to -68.4, 9.4
//   move.facePoint(-64.4, 6.4, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(-64.4, 6.4, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to -68.6, -6.6
//   move.facePoint(-64.6, -14.6, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(-64.6, -14.6, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // b to (no face) -67.8, 32.8
//   move.facePoint(-67.8, 32.8, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(-67.8, 32.8, 7, 12, YKP, TKP, 12, 6);

//   // face to -85.0, 8.0
//   move.facePoint(-80.5, 4.0, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(-80.5, 4.0, 7, 12, YKP, TKP, 12, 6);
//   //delay(1000);

//   // b to (no face) -62.7, 20.1
//   move.facePoint(-62.7, 20.1, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(-62.7, 20.1, 7, 12, YKP, TKP, 12, 6);

//   // b face to -81.4, -8.2
//   move.facePoint(-79.4, -9.2, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(-79.4, -9.2, 12, 12, YKP, TKP, 7, 9);
//   //delay(200);

//   // drop mogo off
//   move.face(45, TKP, TKI, TKD, 12, 8);
//   BackMech.set(0);
//   delay(200);
//   intake.stop();

//   // to -62.2, 4.6
//   move.facePoint(-62.2, 4.6, TKP, TKI, TKD, 7, 6, false);
//   move.to(-62.2, 4.6, 9, 9, YKP, TKP, 12, 6);
//   enableStopper = true;
//   holdRing();
//   intake.spin(forward);

//   // to (load neutral) -70.5, 103.0
//   move.facePoint(-70.5, 103.0, TKP, TKI, TKD, 7, 6, false);
//   move.to(-70.5, 103.0, 9, 9, YKP, TKP, 12, 6);
//   delay(1200);
//   enableStopper = false;
//   continueRing();
//   intake.spin(reverse);
//   delay(100);

//   // b to -67.5, 70.0
//   move.facePoint(-67.5, 71.3, TKP, TKI, TKD, 7, 6, true);
//   move.bto(-67.5, 71.3, 7, 12, YKP, TKP, 7, 3);
//   NeutralLock.set(1);

//   move.face(270, TKP, TKI, TKD, 12, 3);
//   arm.setVelocity(100, percent);
//   arm.spinFor(2700, degrees, false);
//   delay(900);
//   move.straighth(30, 270, YKP, YKD, TKP, TKD, 4, 12, 12, 0);
//   delay(1200);
//   arm.spinFor(-1400, degrees, false);
//   delay(500);
//   NeutralLock.set(0);
//   arm.setStopping(coast);
//   arm.stop();
  


//   autonStop();


// }

// void GordSkills(){
//   delay(7000);
//   Stick.set(0);
//   NeutralLock.set(0);
//   auto_settings();
//   starttime = Brain.timer(msec);
//   intake_lift = false;
//   chassis.setPos(0, 0, 0);
//   TKP = 1.1; //1.0 and 1.1 works good as well (but potential to undershoot) // 1.1 
//   YKP = 2.60; // was 2.4, was 2.65
//   move.cataEnabled = false;

//   enableNeutralStopper = false;
//   enableStopper = false;

//   delay(200);

//   intake.setVelocity(100, percent);
//   move.straighth(-10, chassis.h, YKP, YKD, TKP, TKD, 4, 12, 12, 0);
//   delay(600);
//   intake.spin(forward);
//   delay(700);
//   intake.stop();

//   // to 0.1, 8.9
//   move.to(0.1, 8.9, 7, 12, YKP, TKP, 12, 6);
//   delay(200);

//   // b face to (to mogo) 27.1, 8.5 
//   move.facePoint(30.1, 8.5, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(30.1, 8.5, 4, 12, YKP, TKP, 12, 9);
//   //delay(200);

//   BackMech.set(1);
//   move.voldrive(0, 0);
//   delay(400); // no swing

//   intake.setVelocity(100, percent);
//   intake.spin(forward);

//   // face to 32.8, 36.4
//   move.to(32.8, 36.4, 7, 12, YKP, TKP, 12, 6);

//   // // face to 48.3, 55.4
//   // move.to(48.3, 55.4, 7, 12, YKP, TKP, 12, 6);

//   // face to 81.4, 73.0
//   move.to(81.4, 73.0, 7, 12, YKP, TKP, 12, 6);

//   // b to (no face) 68.5, 65.9
//   move.bto(68.5, 65.9, 7, 12, YKP, TKP, 12, 6);

//   // face to 66.1, 43.2
//   move.to(66.1, 43.2, 7, 12, YKP, TKP, 12, 11); // bl was 6

//   // face to 66.1, 13.9
//   move.to(66.1, 13.9, 7, 12, YKP, TKP, 12, 11); // bl was 6

//   // face to 66.2, -4.7
//   move.to(66.2, -7.7, 7, 12, YKP, TKP, 12, 6);

//   // face to 80.5, 11.5 
//   move.facePoint(80.5, 14.5, TKP, TKI, TKD, 7, 6, false);
//   move.to(80.5, 14.5, 7, 12, YKP, TKP, 12, 6);

//   // // b to 56.2, 23.2
//   move.bto(56.2, 23.2, 7, 12, YKP, TKP*0.8, 7, 6);

//   // b face to 76.4, -6.3
//   move.facePoint(76.4, -6.3, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(76.4, -6.3, 12, 12, YKP, TKP, 7, 9);
//   //delay(200);

//   // mogo drop off
//   move.face(315, TKP, TKI, TKD, 12, 8);
//   BackMech.set(0);
//   delay(200);
//   intake.stop();

//   // SECOND MOGO /////////////////////////////////////////////////////////
//   // to (no face) 63.8, 6.4
//   move.to(63.8, 3.4, 7, 12, YKP, TKP, 12, 6);
//   delay(200);

//   // face to -6.0, 5.2
//   // was 8.4 on y
//   move.facePoint(-6.0, 2.4, TKP*1.2, TKI, TKD*1.3, 7, 6, false);
//   delay(200);
//   move.to(-6.0, 2.4, 6, 12, YKP, TKP*1.2, 12, 6);
//   delay(200);

//   // face b to -37.6, 6.3
//   // was 8.4 on y
//   move.facePoint(-37.6, 2.4, TKP, TKI, TKD, 7, 6, true);
//   delay(200);
//   //chassis.setPos(-2.0, 3.9, chassis.h); // RESET (x is offset to the right to push entire route to the left)
//   move.bto(-37.6, 2.4, 4, 12, YKP, TKP, 12, 7);
//   //delay(200);

//   // move.PPSbto(-33.8, 3.9, 6, 6, YKP, TKP*0.6, 12, 3/*BL*/, 40/*RAng*/, 6/*TE*/, 22/*Look*/);
//   // BackMech.set(1);
//   // delay(200);

//   // clamp
//   BackMech.set(1);
//   move.voldrive(0, 0);
//   delay(400); // no swing
//   intake.setVelocity(100, percent);
//   intake.spin(forward);

//   // face to -35.4, 34.0
//   move.to(-35.4, 34.0, 7, 12, YKP, TKP, 12, 6);

//   // face to -63.7, 37.3
//   move.to(-63.7, 37.3, 7, 12, YKP, TKP, 12, 6);

//   // face to -81.7, 69.7
//   move.to(-81.7, 69.7, 7, 12, YKP, TKP, 12, 6);

//   // b to (no face) -64.5, 45.9 
//   move.bto(-64.5, 45.9, 7, 12, YKP, TKP, 12, 6);

//   // face to -68.4, 9.4
//   move.to(-64.4, 9.4, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to -68.6, -6.6
//   move.to(-64.6, -6.6, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // // b to (no face) -67.8, 32.8
//   // move.bto(-67.8, 32.8, 7, 12, YKP, TKP, 12, 6);

//   // face to -85.0, 8.0
//   move.to(-80.5, 13.0, 7, 12, YKP, TKP, 12, 6);
//   //delay(1000);

//   // // b to (no face) -62.7, 20.1
//   move.bto(-62.7, 20.1, 7, 12, YKP, TKP, 12, 6);

//   // b face to -81.4, -8.2
//   move.facePoint(-79.4, -9.2, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(-79.4, -9.2, 12, 12, YKP, TKP, 7, 0);
//   delay(2500);
//   //delay(200);

//   // drop mogo off
//   move.face(45, TKP, TKI, TKD, 12, 8);
//   BackMech.set(0);
//   delay(200);
//   intake.stop();

//   autonStop();
// }


// void refinedSkills(){
//   delay(7000);
//   Stick.set(0);
//   NeutralLock.set(0);
//   auto_settings();
//   starttime = Brain.timer(msec);
//   intake_lift = false;
//   chassis.setPos(0, 0, 0);
//   //test this
//   TKP = 0.389;
//   TKI = 0.012;
//   TKD = 2.715;
//   YKP = 1.5;
//   YKI = 0;
//   YKD = 8.5;

//   move.cataEnabled = false;
//   enableNeutralStopper = false;
//   enableStopper = false;

//   intake.setVelocity(100, percent);
//   move.straighth(-10, chassis.h, YKP, YKD, TKP, TKD, 2, 12, 12, 0);
//   delay(1000);
//   intake.spin(forward);
//   delay(700);
//   intake.stop();

//   // pps to -0.5, 9.3 relative ang 90
//   move.PPSto(-0.5, 9.3, 12, 12, YKP, TKP, 12, 3/*BL*/, 90, 3, 14);

//   // b pps to 26.1, 7.2 relative ang 0
//   move.PPSbto(26.1, 7.2, 4, 4, YKP, TKP*0.6, 12, 3/*BL*/, 15, 8, 22);
//   BackMech.set(1);
//   delay(200);
//   intake.setVelocity(100, percent);
//   intake.spin(forward);

//   // pps to 34.0, 39.8
//   move.facePoint(34.0, 39.8, TKP, TKI, TKD, 12, 10, false);
//   //move.PPSto(34.0, 39.8, 8, 8, YKP, TKP, 12, 8/*BL*/, 5/*RAng*/, 6/*TE*/, 22/*Look*/);
//   move.to(34.0, 39.8, 12, 12, YKP, TKP, 12, 8);
//   enableStopper = true;
//   holdRing();

//   // pps to (no ring) (slow for eject) 53.0, 62.0
//   move.PPSto(53.0, 62.0, 3, 3, YKP, TKP, 12, 0/*BL*/, 35/*RAng*/, 6/*TE*/, 22/*Look*/);
//   delay(1100);
//   enableStopper = false;
//   continueRing();
//   intake.spin(reverse);
//   delay(800);
//   intake.spin(forward);

//   // pps to 65.8, 109.5
//   move.PPSto(65.8, 109.5, 6, 6, YKP, TKP, 12, 3/*BL*/, 25/*RAng was 55*/, 6/*TE*/, 22/*Look*/);
//   NeutralLock.set(1);

//   // b pps to 56.3, 75.3
//   move.facePoint(56.3, 75.3, TKP, TKI, TKD, 12, 10, true);
//   move.PPSbto(56.3, 75.3, 6, 6, YKP, TKP, 12, 3/*BL*/, 5/*RAng*/, 6/*TE*/, 16/*Look*/);
//   // make sure its slow enough so previous ring is scored

//   // pps to (close stake ring) 80.1, 72.9 and wait a bit
//   move.facePoint(80.1, 72.9, TKP, TKI, TKD, 12, 10, false);
//   move.PPSto(80.1, 72.9, 4, 4, YKP, TKP, 12, 3/*BL*/, 5/*RAng*/, 6/*TE*/, 22/*Look*/);
//   enableStopper = true;
//   holdRing();
//   delay(100);

//   // neutral stake ring 1
//   // b face and b pps to 68.7, 72.0
//   move.facePoint(68.7, 72.0, TKP, TKI, TKD, 12, 10, true);
//   intake.stop();
//   arm.setVelocity(100, percent);
//   arm.spinFor(2700, degrees, false);
//   move.PPSbto(68.7, 72.0, 6, 6, YKP, TKP, 12, 3/*BL*/, 5/*RAng*/, 6/*TE*/, 16/*Look*/);
//   delay(500);
//   // face and pps to 85.9, 71.7
//   move.facePoint(85.9, 72.4, TKP, TKI, TKD, 12, 3, false);
//   //move.PPSto(85.9, 72.4, 6, 6, YKP, TKP, 12, 3/*BL*/, 2/*RAng*/, 6/*TE*/, 22/*Look*/);
//   move.to(85.9, 72.4, 12, 12, YKP, TKP, 12, 3);

//   move.face(90, TKP, TKI, TKD, 12, 3);
//   move.voldrive(4, 4);
//   delay(700);
//   arm.spinFor(-1400, degrees, false);
//   delay(500);
//   NeutralLock.set(0);
//   arm.setStopping(coast);
//   arm.stop();

//   // back off and load 2nd ring
//   move.PPSbto(70.7, 71.9, 6, 6, YKP, TKP, 12, 3/*BL*/, 5/*RAng*/, 6/*TE*/, 16/*Look*/);
//   delay(500);
//   enableStopper = false;
//   continueRing();
//   intake.spin(forward);

//   // pps to 69.7, 33.6
//   move.facePoint(64.7, 33.6, TKP, TKI, TKD, 12, 10, false); 
//   move.PPSto(64.7, 33.6, 12, 12, YKP, TKP, 12, 3/*BL*/, 5/*RAng*/, 6/*TE*/, 22/*Look*/);

//   // pps to 69.3, -11.3
//   move.PPSto(64.3, -9.3, 6, 6, YKP, TKP, 12, 3/*BL*/, 5/*RAng*/, 6/*TE*/, 22/*Look*/);
//   delay(400);

//   // face pps to 81.5, 8.7
//   //move.facePoint(81.5, 12.7, TKP, TKI, TKD, 12, 10, false); 
//   move.PPSto(81.5, 16.7, 6, 6, YKP, TKP*0.6, 12, 3/*BL*/, 20/*RAng*/, 6/*TE*/, 22/*Look*/);
//   delay(300);

//   // face pps b to 79.5, 2.3
//   move.PPSbto(79.5, 2.3, 6, 6, YKP, TKP, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 22/*Look*/);
//   move.face(-34.1, TKP, TKI, TKD, 12, 3);
//   BackMech.set(0);
//   move.voldrive(-4, -4);
//   delay(900);

//   // field cross
//   // pps to -0.7, 7.8
//   // move.PPSto(-0.7, 7.2, 12, 12, YKP, TKP, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 22/*Look*/);
//   // delay(400);

//   // SECOND GOAL /////////////////////////////////////////////////////////////////////////////////////////

//   // face b pps to -32.6, 8.6
//   // move.facePoint(-32.6, 7.2, TKP, TKI, TKD, 12, 3, true);
//   // move.PPSbto(-32.6, 7.2, 4, 4, YKP, TKP*0.6, 12, 3/*BL*/, 85, 8, 22);
//   move.PPSbto(-28.6, 7.2, 6, 6, YKP, TKP*0.6, 12, 3/*BL*/, 40/*RAng*/, 6/*TE*/, 22/*Look*/);
//   BackMech.set(1);
//   delay(200);
//   intake.setVelocity(100, percent);
//   intake.spin(forward);

//   // pps to -29.8, 45.9
//   move.facePoint(-29.8, 32.9, TKP, TKI, TKD, 12, 10, false);
//   //move.PPSto(34.0, 39.8, 8, 8, YKP, TKP, 12, 8/*BL*/, 5/*RAng*/, 6/*TE*/, 22/*Look*/);
//   move.to(-29.8, 32.9, 12, 12, YKP, TKP, 12, 8);
//   enableStopper = true;
//   holdRing();

//   // pps to (no ring) (slow for eject) -55.8, 75.2
//   move.PPSto(-55.8, 62.2, 3, 3, YKP, TKP, 12, 0/*BL*/, 15/*RAng*/, 6/*TE*/, 22/*Look*/);
//   delay(1100);
//   enableStopper = false;
//   continueRing();
//   intake.spin(reverse);
//   delay(800);
//   intake.spin(forward);

//   // pps to -61.7, 111.6
//   move.PPSto(-66.7, 111.6, 6, 6, YKP, TKP, 12, 3/*BL*/, 15/*RAng was 55*/, 6/*TE*/, 22/*Look*/);
//   NeutralLock.set(1);

//   // b pps to -54.6, 77.6
//   move.facePoint(-56.6, 69.6, TKP, TKI, TKD, 12, 3, true);
//   move.PPSbto(-56.6, 69.6, 6, 6, YKP, TKP, 12, 3/*BL*/, 5/*RAng*/, 6/*TE*/, 22/*Look*/);
//   // make sure its slow enough so previous ring is scored

//   // pps to (close stake ring) -78.1, 75.0 and wait a bit
//   move.facePoint(-78.1, 67.0, TKP, TKI, TKD, 12, 3, false);
//   move.PPSto(-78.1, 67.0, 4, 4, YKP, TKP, 12, 3/*BL*/, 5/*RAng*/, 6/*TE*/, 22/*Look*/);
//   enableStopper = true;
//   holdRing();
//   delay(100);

//   // neutral stake ring 2
//   // b face and b pps to -62.4, 73.4
//   move.facePoint(-62.4, 67.4, TKP, TKI, TKD, 12, 10, true);
//   intake.stop();
//   arm.setVelocity(100, percent);
//   arm.spinFor(2700, degrees, false);
//   move.PPSbto(-62.4, 67.4, 6, 6, YKP, TKP, 12, 3/*BL*/, 5/*RAng*/, 6/*TE*/, 8/*Look*/);
//   delay(500);
//   // face and pps to -80.9, 74.8
//   move.facePoint(-80.9, 65.8, TKP, TKI, TKD, 12, 3, false);
//   //move.PPSto(85.9, 72.4, 6, 6, YKP, TKP, 12, 3/*BL*/, 2/*RAng*/, 6/*TE*/, 22/*Look*/);
//   move.to(-80.9, 65.8, 6, 6, YKP, TKP, 12, 3);

//   move.face(270, TKP, TKI, TKD, 12, 3);
//   move.voldrive(4, 4);
//   delay(700);
//   arm.spinFor(-1400, degrees, false);
//   delay(500);
//   NeutralLock.set(0);
//   arm.setStopping(coast);
//   arm.stop();

//   // back off and load 2nd ring
//   move.PPSbto(-63.9, 65.8, 6, 6, YKP, TKP*0.4, 12, 3/*BL*/, 5/*RAng*/, 6/*TE*/, 16/*Look*/);
//   delay(500);
//   enableStopper = false;
//   continueRing();
//   intake.spin(forward);

//   // pps to -62.7, 38.4
//   move.facePoint(-63.7, 38.4, TKP, TKI, TKD, 12, 10, false); 
//   move.PPSto(-63.7, 38.4, 12, 12, YKP, TKP, 12, 3/*BL*/, 5/*RAng*/, 6/*TE*/, 22/*Look*/);

//   // pps to -64.9, -8.1
//   move.PPSto(-64.9, -14.1, 6, 6, YKP, TKP, 12, 3/*BL*/, 5/*RAng*/, 6/*TE*/, 22/*Look*/);
//   delay(400);

//   // face pps to -82.4, 19.2
//   //move.facePoint(81.5, 12.7, TKP, TKI, TKD, 12, 10, false); 
//   move.PPSto(-82.4, 7.2, 6, 6, YKP, TKP, 12, 6/*BL*/, 10/*RAng*/, 6/*TE*/, 22/*Look*/);
//   delay(500);

//   // face b to -80.3, 4.9
//   move.PPSbto(-75.3, 2.9, 6, 6, YKP, TKP, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 22/*Look*/);
//   move.face(33.1, TKP, TKI, TKD, 12, 3);
//   BackMech.set(0);
//   move.voldrive(-4, -4);
//   delay(800);
//   intake.setStopping(coast);
//   intake.stop();

//   // pps to -7.5, 63.2
//   move.PPSto(-7.5, 63.2, 6, 6, YKP, TKP, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 22/*Look*/);
//   // pps to 23.5, 98.5, slow and carefully
//   move.PPSto(23.5, 98.5, 4, 4, YKP, TKP, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 22/*Look*/);
//   // pps to 33.3, 108.5 while intaking at 30 and hold ring is activated
//   intake.setVelocity(60, percent);
//   intake.spin(forward);
//   enableStopper = true;
//   holdRing();
//   move.PPSto(33.3, 108.5, 4, 4, YKP, TKP, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 22/*Look*/);
//   delay(800);
//   // b pps to -3.8, 133.6
//   move.PPSbto(-3.8, 133.6, 4, 4, YKP, TKP*0.6, 12, 3/*BL*/, 15, 8, 22);
//   BackMech.set(1);
//   delay(200);


//   autonStop();

// }

// void doubleMogo(){
//   //delay(7000);
//   Stick.set(0);
//   NeutralLock.set(0);
//   auto_settings();
//   starttime = Brain.timer(msec);
//   intake_lift = false;
//   chassis.setPos(0, 0, 0);
//   //test this
//   TKP = 0.389;
//   TKI = 0.012;
//   TKD = 2.715;
//   YKP = 1.5;
//   YKI = 0;
//   YKD = 8.5;
//   move.cataEnabled = false;
//   enableNeutralStopper = false;
//   enableStopper = false;

//   Stick.set(1);
//   StickMech.set(0);
//   enableStopper = true;
//   holdRing();
//   intake.setVelocity(100, percent);
//   intake.spin(forward);

//   // pps to 21.4, 53.9
//   move.PPSto(21.4, 53.9, 12, 12, YKP, TKP*0.6, 12, 14/*BL*/, 15/*RAng*/, 6/*TE*/, 38/*Look*/);
//   move.face(25, TKP, TKI, TKD, 12, 9);
//   delay(50);
//   StickMech.set(1);

//   // b pps to 16.9, 24.9
//   move.PPSbto(20.9, 16.9, 12, 12, YKP, TKP, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 22/*Look*/);
//   delay(300);
//   StickMech.set(0);
//   move.face(180, TKP, TKI, TKD, 12, 4);
//   Stick.set(0);

//   // b pps to 17.8, 37.6
//   move.PPSbto(17.8, 37.6, 12, 12, YKP, TKP, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 22/*Look*/);
//   move.voldrive(-3, -3);
//   delay(300);
//   BackMech.set(1);
//   move.voldrive(2, 2);
//   delay(200);
//   intake.spinFor(900, degrees, false);

//   // b pps to 18.7, 6.9
//   move.PPSbto(18.7, 6.9, 9, 12, YKP, TKP, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 22/*Look*/);
//   move.face(90, TKP, TKI, TKD, 12, 9);
//   BackMech.set(0);

//   // pps to 19.0, 19.5
//   move.PPSto(19.0, 19.5, 12, 12, YKP, TKP*0.6, 12, 14/*BL*/, 15/*RAng*/, 6/*TE*/, 38/*Look*/);

//   // b pps to -6.2, 33.3
//   move.PPSbto(-6.2, 38.3, 7, 12, YKP, TKP, 12, 3/*BL*/, 10/*RAng*/, 12/*TE*/, 22/*Look*/);
//   move.voldrive(-2, -2);
//   delay(200);
//   BackMech.set(1);
//   intake.spin(forward);

//   // pps to -22.9, -5.2
//   move.PPSto(-22.9, -5.2, 9, 12, YKP, TKP*0.6, 12, 14/*BL*/, 15/*RAng*/, 6/*TE*/, 30/*Look*/);

//   // b pps to 7.7, 22.1
//   move.PPSbto(7.7, 22.1, 7, 12, YKP, TKP, 12, 3/*BL*/, 10/*RAng*/, 12/*TE*/, 22/*Look*/);
//   Stick.set(1);

//   // pps to 37.4, 4.0
//   move.PPSto(37.4, -2.0, 9, 12, YKP, TKP, 12, 3/*BL*/, 15/*RAng*/, 6/*TE*/, 30/*Look*/);

//   move.face(180, TKP, TKI, TKD, 12, 0);
//   delay(500);
//   move.face(270, TKP, TKI, TKD, 12, 9);
//   Stick.set(0);  





//   autonStop();

//   // b pps to -8.4, 38.9
//   move.PPSbto(-8.4, 38.9, 6, 6, YKP, TKP, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 22/*Look*/);
//   BackMech.set(1);
//   move.voldrive(2, 2);
//   delay(200);
//   intake.spinFor(900, degrees, true);



//   // pps to -15.2, -8.3
//   move.PPSbto(-15.2, -8.3, 12, 12, YKP, TKP*0.6, 12, 0/*BL*/, 15/*RAng*/, 6/*TE*/, 38/*Look*/);
//   delay(200);
//   BackMech.set(0);
//   // enableStopper = false;
//   // continueRing();
//   // intake.spin(forward);
//   // delay(1000);

//   // arm.setVelocity(100, percent);
//   // arm.setStopping(hold);
//   // arm.spinFor(1000, degrees, false);

//   // // pps to 12.4, -16.6
//   // move.PPSto(12.4, -16.6, 7, 7, YKP, TKP*0.6, 12, 4/*BL*/, 15/*RAng*/, 6/*TE*/, 38/*Look*/);
//   // intake.spin(reverse);

//   // // straight h at 102.1 degrees
//   // move.face(102.1, TKP, TKI, TKD, 12, 3);
//   // move.straighth(50, 102.1, YKP, YKD, TKP, TKD, 8, 8, 12, 0);
//   // delay(800);
//   // move.straighth(-3, 102.1, YKP, YKD, TKP, TKD, 8, 8, 12, 2);
//   // move.straighth(50, 102.1, YKP, YKD, TKP, TKD, 8, 8, 12, 0);
//   // delay(800);
//   // intake.spin(forward);
//   // delay(800);
//   // move.straighth(-7, 102.1, YKP, YKD, TKP, TKD, 8, 8, 12, 2);
//   // delay(800);

//   // move.face(270, TKP, TKI, TKD, 12, 8);
//   // arm.setStopping(coast);
//   // arm.stop();
//   // BackMech.set(0);

//   // // pps to -24.4, -2.2
//   // move.PPSto(-24.4, -2.2, 7, 7, YKP, TKP*0.6, 12, 4/*BL*/, 15/*RAng*/, 6/*TE*/, 38/*Look*/);

//   // // fake color sort (30 intake, double hold)
//   // enableStopper = true;
//   // holdRing();
//   // intake.setVelocity(40, percent);
//   // move.straighth(20, chassis.h, YKP, YKD, TKP, TKD, 3, 3, 12, 2);
//   // delay(600);
//   // intake.setVelocity(100, percent);
//   // delay(500);
//   // enableStopper = false;
//   // continueRing();
//   // intake.spinFor(300, degrees, true);

//   // // pps to 15.7, 14.9
//   // move.PPSto(15.7, 14.9, 7, 7, YKP, TKP*0.6, 12, 4/*BL*/, 15/*RAng*/, 6/*TE*/, 38/*Look*/);

//   // // b pps to 14.6, 36.7
//   // move.PPSbto(14.6, 36.7, 6, 6, YKP, TKP, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 22/*Look*/);
//   // BackMech.set(1);


//   autonStop();

// }

// void PPSSkills(){
//   delay(7000);
//   Stick.set(0);
//   NeutralLock.set(0);
//   auto_settings();
//   starttime = Brain.timer(msec);
//   intake_lift = false;
//   chassis.setPos(0, 0, 0);
//   TKP = 1.1*0.7; //was 1.1*0.7
//   YKP = 2.60; // was 2.60
//   // test this
//   // TKP = 0.389;
//   // TKI = 0.012;
//   // TKD = 2.715;
//   // YKP = 1.5;
//   // YKI = 0;
//   // YKD = 8.5;
//   move.cataEnabled = false;

//   enableNeutralStopper = false;
//   enableStopper = false;

//   delay(200);

//   intake.setVelocity(100, percent);
//   move.straighth(-10, chassis.h, YKP, YKD, TKP, TKD, 2, 12, 12, 0);
//   delay(1000);
//   intake.spin(forward);
//   delay(700);
//   intake.stop();

//   // pps to -0.5, 9.3 relative ang 90
//   move.PPSto(-0.5, 9.3, 12, 12, YKP, TKP, 12, 3/*BL*/, 90, 3, 14);

//   // b pps to 30.8, 10.4 relative ang 0
//   move.PPSbto(30.8, 10.4, 4, 4, YKP, TKP*0.6, 12, 3/*BL*/, 15, 8, 22);
//   BackMech.set(1);
//   delay(200);
//   intake.setVelocity(100, percent);
//   intake.spin(forward);

//   // pps to 32.9, 44.5
//   move.PPSto(32.9, 44.5, 12, 12, YKP, TKP, 12, 8/*BL*/, 20/*RAng*/, 6/*TE*/, 22/*Look*/);
//   enableStopper = true;
//   holdRing();

//   // pps to (no ring) (slow for eject) 54.6, 73.1
//   move.PPSto(54.6, 73.1, 6, 6, YKP, TKP, 12, 0/*BL*/, 35/*RAng*/, 6/*TE*/, 22/*Look*/);
//   delay(1100);
//   enableStopper = false;
//   continueRing();
//   intake.spin(reverse);
//   delay(800);
//   intake.spin(forward);

//   // pps to 66.5, 108.2
//   move.PPSto(66.5, 108.2, 6, 6, YKP, TKP, 12, 6/*BL*/, 55/*RAng was 55*/, 6/*TE*/, 22/*Look*/);
//   NeutralLock.set(1);
//   // arm.setVelocity(50, percent);
//   // arm.spinFor(1800, degrees, false);

//   // // pps to 25.4, 109.9
//   // move.PPSto(25.4, 109.9, 6, 6, YKP, TKP, 12, 6/*BL*/, 20/*RAng*/, 6/*TE*/, 22/*Look*/);
//   // delay(500);

//   // b pps to 55.4, 72.1
//   move.PPSbto(52.4, 72.1, 6, 6, YKP, TKP, 12, 9/*BL*/, 10/*RAng*/, 6/*TE*/, 16/*Look*/);
//   // make sure its slow enough so previous ring is scored

//   // pps to (close stake ring) 75.9, 72.3 and wait a bit
//   move.PPSto(75.9, 72.3, 4, 4, YKP, TKP, 12, 3/*BL*/, 50/*RAng*/, 6/*TE*/, 22/*Look*/);
//   delay(1200);

//   // neutral stake ring 1
//   // b pps to 66.8, 72.4
//   intake.stop();
//   arm.setVelocity(100, percent);
//   arm.spinFor(2700, degrees, false);
//   move.PPSbto(68.8, 72.4, 6, 6, YKP, TKP*1.1, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 16/*Look*/);
//   delay(500);
//   // pps to 81.9, 72.4
//   move.PPSto(81.9, 72.4, 6, 6, YKP, TKP*1.23, 12, 3/*BL*/, 4/*RAng*/, 6/*TE*/, 22/*Look*/);

//   move.face(90, TKP, TKI, TKD, 12, 3);
//   move.voldrive(4, 4);
//   delay(700);
//   arm.spinFor(-1400, degrees, false);
//   delay(700);
//   NeutralLock.set(0);
//   arm.setStopping(coast);
//   arm.stop();

//   // back off and load 2nd ring
//   move.PPSbto(66.8, 72.4, 6, 6, YKP, TKP, 12, 9/*BL*/, 10/*RAng*/, 6/*TE*/, 16/*Look*/);
//   delay(500);
//   enableStopper = false;
//   continueRing();
//   intake.spin(forward);

//   // pps to 62.8, -7.6
//   move.PPSto(62.8, -7.6, 6, 6, YKP, TKP, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 22/*Look*/);

//   // pps to 81.8, 12.0
//   move.PPSto(81.8, 12.0, 6, 6, YKP, TKP, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 22/*Look*/);

//   // pps b to 76.2, -1.5
//   move.PPSbto(76.2, -1.5, 6, 6, YKP, TKP, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 22/*Look*/);
//   move.face(315, TKP, TKI, TKD, 12, 3);
//   BackMech.set(0);
//   move.voldrive(-4, -4);
//   delay(400);
//   move.voldrive(3, 3);
//   delay(400);

//   // SECOND MOBILE GOAL
//   // pps b to -33.8, 11.9
//   move.PPSbto(-33.8, 3.9, 6, 6, YKP, TKP*0.6, 12, 3/*BL*/, 40/*RAng*/, 6/*TE*/, 22/*Look*/);
//   BackMech.set(1);
//   delay(200);

//   // // pps to -34.6, 42.3
//   // move.PPSto(-34.6, 42.3, 12, 12, YKP, TKP, 12, 8/*BL*/, 12/*RAng*/, 6/*TE*/, 22/*Look*/);
//   // intake.setVelocity(100, percent);
//   // intake.spin(forward);

//   // // // pps to (no ring) -59.9, 71.7
//   // // move.PPSto(-59.9, 71.7, 6, 6, YKP, TKP, 12, 0/*BL*/, 20/*RAng*/, 6/*TE*/, 22/*Look*/);
//   // // delay(1100);
//   // // enableStopper = false;
//   // // continueRing();
//   // // intake.spin(reverse);
//   // // delay(800);
//   // // intake.spin(forward);

//   // // // pps to -68.5, 109.8
//   // // move.PPSto(-72.5, 109.8, 6, 6, YKP, TKP, 12, 6/*BL*/, 35/*RAng was 55*/, 6/*TE*/, 22/*Look*/);
//   // // NeutralLock.set(1);

//   // // // b pps to -55.9, 70.1
//   // // move.PPSbto(-58.9, 68.1, 6, 6, YKP, TKP, 12, 9/*BL*/, 10/*RAng*/, 6/*TE*/, 16/*Look*/);

//   // // pps to -82.7, 70.6 (close stake ring)
//   // move.PPSto(-80.7, 68.6, 4, 4, YKP, TKP, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 22/*Look*/);
//   // delay(1200);

//   // // // b pps to -68.0, 70.1 neutral stake 2
//   // // intake.stop();
//   // // arm.setVelocity(100, percent);
//   // // arm.spinFor(2700, degrees, false);
//   // // move.PPSbto(-68.0, 65.1, 6, 6, YKP, TKP*1.1, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 16/*Look*/);
//   // // delay(500);

//   // // // pps to -84.2, 71.2
//   // // // move.PPSto(-84.2, 66.5, 6, 6, YKP, TKP*1.23, 12, 3/*BL*/, 7/*RAng*/, 6/*TE*/, 22/*Look*/);
//   // // move.facePoint(-84.2, 66.5, TKP, TKI, TKD, 12, 3, false);
//   // // move.to(-84.2, 66.5, 12, 12, YKP, TKP, 12, 3);

//   // // move.face(270, TKP, TKI, TKD, 12, 3);
//   // // move.voldrive(4, 4);
//   // // delay(700);
//   // // arm.spinFor(-900, degrees, false);
//   // // delay(300);
//   // // NeutralLock.set(0);
//   // // arm.setStopping(coast);
//   // // arm.stop();

//   // // // pps b to 
//   // // move.PPSbto(-68.0, 68.1, 6, 6, YKP, TKP*1.1, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 16/*Look*/);
//   // // delay(500);
//   // // enableStopper = false;
//   // // continueRing();
//   // // intake.spin(forward);

//   // // pps to -69.3, -11.1
//   // move.PPSto(-73.3, -11.1, 6, 6, YKP, TKP, 12, 3/*BL*/, 5/*RAng*/, 6/*TE*/, 22/*Look*/);

//   // // pps to -86.1, 6.8
//   // move.PPSto(-86.1, 6.8, 6, 6, YKP, TKP, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 22/*Look*/);

//   // face to -35.4, 34.0
//   move.to(-35.4, 34.0, 7, 12, YKP, TKP, 12, 6);

//   // face to -63.7, 37.3
//   move.to(-63.7, 33.3, 7, 12, YKP, TKP, 12, 6);

//   // face to -81.7, 69.7
//   move.to(-83.7, 69.7, 7, 12, YKP, TKP, 12, 6);

//   // b to (no face) -64.5, 45.9 
//   move.bto(-64.5, 45.9, 7, 12, YKP, TKP, 12, 6);

//   // face to -68.4, 9.4
//   move.to(-67.4, 9.4, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to -68.6, -6.6
//   move.to(-67.6, -6.6, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // // b to (no face) -67.8, 32.8
//   // move.bto(-67.8, 32.8, 7, 12, YKP, TKP, 12, 6);

//   // face to -85.0, 8.0
//   move.to(-80.5, 12.0, 7, 12, YKP, TKP, 12, 6);
//   //delay(1000);

//   // b pps to -79.4, -2.7
//   move.PPSbto(-79.4, -2.7, 6, 6, YKP, TKP, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 22/*Look*/);
//   move.face(45, TKP, TKI, TKD, 12, 3);
//   BackMech.set(0);
//   move.voldrive(-4, -4);
//   delay(400);





//   autonStop();

//   // relative ang is angle b/w x-axis and the next point, robot's first heading is classified as 0 
//   // TurnEnd is distance to the last point where the robot finishes turning, and starts going straight
//   // T is lookahead
//   move.PPSto(32.8, 36.4, 6, 6, YKP, TKP, 12, 6, 25, 3, 8);

// autonStop();
// }

// void HWSkills(){
//   //delay(7000);
//   Stick.set(0);
//   NeutralLock.set(0);
//   auto_settings();
//   starttime = Brain.timer(msec);
//   intake_lift = false;
//   chassis.setPos(0, 0, 0);
//   TKP = 1.1; //1.0 and 1.1 works good as well (but potential to undershoot) // 1.1 
//   YKP = 2.60; // was 2.4, was 2.65
//   move.cataEnabled = false;

//   enableNeutralStopper = false;
//   enableStopper = false;

//   delay(200);

//   intake.setVelocity(100, percent);
//   move.straighth(-10, chassis.h, YKP, YKD, TKP, TKD, 4, 12, 12, 0);
//   delay(600);
//   intake.spin(forward);
//   delay(700);
//   intake.stop();

//   // to 0.1, 8.9
//   move.to(0.1, 8.9, 7, 12, YKP, TKP, 12, 6);
//   delay(200);

//   // b face to (to mogo) 27.1, 8.5 
//   move.facePoint(30.1, 8.5, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(30.1, 8.5, 4, 12, YKP, TKP, 12, 9);
//   //delay(200);

//   BackMech.set(1);
//   move.voldrive(0, 0);
//   delay(400); // no swing

//     intake.setVelocity(100, percent);
//   intake.spin(forward);

//   // face to 32.8, 36.4
//   move.to(32.8, 36.4, 7, 12, YKP, TKP, 12, 6);

//   // // face to 48.3, 55.4
//   // move.to(48.3, 55.4, 7, 12, YKP, TKP, 12, 6);

//   // face to 81.4, 73.0
//   move.to(81.4, 73.0, 7, 12, YKP, TKP, 12, 6);

//   // b to (no face) 68.5, 65.9
//   move.bto(68.5, 65.9, 7, 12, YKP, TKP, 12, 6);

//   // face to 66.1, 43.2
//   move.to(66.1, 43.2, 7, 12, YKP, TKP, 12, 11); // bl was 6

//   // face to 66.1, 13.9
//   move.to(66.1, 13.9, 7, 12, YKP, TKP, 12, 11); // bl was 6

//   // face to 66.2, -4.7
//   move.to(66.2, -7.7, 7, 12, YKP, TKP, 12, 6);

//   // face to 80.5, 11.5 
//   move.facePoint(80.5, 14.5, TKP, TKI, TKD, 7, 6, false);
//   move.to(80.5, 14.5, 7, 12, YKP, TKP, 12, 6);

//   // // b to 56.2, 23.2
//   move.bto(56.2, 23.2, 7, 12, YKP, TKP*0.8, 7, 6);

//   // b face to 76.4, -6.3
//   move.facePoint(76.4, -6.3, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(76.4, -6.3, 12, 12, YKP, TKP, 7, 9);
//   //delay(200);

//   // mogo drop off
//   move.face(315, TKP, TKI, TKD, 12, 8);
//   BackMech.set(0);
//   delay(200);
//   intake.stop();

//   // SECOND MOGO /////////////////////////////////////////////////////////
//   // to (no face) 63.8, 6.4
//   move.to(63.8, 3.4, 7, 12, YKP, TKP, 12, 6);
//   delay(200);

//   // face to -6.0, 5.2
//   // was 8.4 on y
//   move.facePoint(-6.0, 3.4, TKP*1.2, TKI, TKD, 7, 6, false);
//   delay(200);
//   move.to(-6.0, 3.4, 6, 12, YKP, TKP*1.2, 12, 6);
//   delay(200);

//   // face b to -37.6, 6.3
//   // was 8.4 on y
//   move.facePoint(-37.6, 3.4, TKP, TKI, TKD*1.3, 7, 6, true);
//   delay(200);
//   //chassis.setPos(-2.0, 3.9, chassis.h); // RESET (x is offset to the right to push entire route to the left)
//   move.bto(-37.6, 3.4, 4, 12, YKP, TKP, 12, 7);
//   //delay(200);

//   // move.PPSbto(-33.8, 3.9, 6, 6, YKP, TKP*0.6, 12, 3/*BL*/, 40/*RAng*/, 6/*TE*/, 22/*Look*/);
//   // BackMech.set(1);
//   // delay(200);

//   // clamp
//   BackMech.set(1);
//   move.voldrive(0, 0);
//   delay(400); // no swing
//   intake.setVelocity(100, percent);
//   intake.spin(forward);

//   // face to -35.4, 34.0
//   move.to(-35.4, 34.0, 7, 12, YKP, TKP, 12, 6);

//   // face to -63.7, 37.3
//   move.to(-63.7, 37.3, 7, 12, YKP, TKP, 12, 6);

//   // face to -81.7, 69.7
//   move.to(-81.7, 69.7, 7, 12, YKP, TKP, 12, 6);

//   // b to (no face) -64.5, 45.9 
//   move.bto(-64.5, 45.9, 7, 12, YKP, TKP, 12, 6);

//   // face to -68.4, 9.4
//   move.to(-64.4, 9.4, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to -68.6, -6.6
//   move.to(-64.6, -6.6, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // // b to (no face) -67.8, 32.8
//   // move.bto(-67.8, 32.8, 7, 12, YKP, TKP, 12, 6);

//   // face to -85.0, 8.0
//   move.to(-80.5, 13.0, 7, 12, YKP, TKP, 12, 6);
//   //delay(1000);

//   // // b to (no face) -62.7, 20.1
//   move.bto(-62.7, 20.1, 7, 12, YKP, TKP, 12, 6);

//   // b face to -81.4, -8.2
//   move.facePoint(-81.4, -8.2, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(-81.4, -8.2, 5, 12, YKP, TKP, 7, 0);
//   delay(2500);
//   //delay(200);

//   // drop mogo off
//   move.face(45, TKP, TKI, TKD, 12, 8);
//   BackMech.set(0);
//   delay(200);
//   intake.stop();

//     // REDO THIRD MOGO /////////////////////////////////////////////////////
//   move.to(-69.6, 5.0, 12, 12, YKP, TKP*1.2, 12, 6);
//   delay(200);

//   // to -65.1, 75.5 
//   move.to(-65.1, 75.5, 12, 12, YKP, TKP, 12, 6); 

//   // to -28.6, 107.0 (hold ring)
//   enableStopper = true;
//   holdRing();
//   move.to(-28.6, 109.0, 7, 12, YKP, TKP*1.2, 12, 6); 
//   delay(300); // was 800

//   // b face to clamp -0.4, 130.5
//   move.facePoint(-0.4, 138.0, TKP*1.2, TKI, TKD, 7, 6, true);
//   delay(200);
//   move.bto(-0.4, 138.0, 4.2, 12, YKP, TKP*1.2, 12, 6);
//   BackMech.set(1);
//   move.voldrive(0, 0);
//   delay(400); // no swing
//   intake.setVelocity(100, percent);
//   enableStopper = false;
//   continueRing();

//   // face to -76.0, 94.9
//   intake.spin(forward);
//   // next 4 lines are to go to left ring and back
//   // move.to(-79.0, 94.9, 7, 12, YKP, TKP*1.2, 12, 6);
//   // // b face to 4.3, 130.8
//   // move.bto(4.3, 140.8, 7, 12, YKP, TKP, 12, 6);
//   // // face to 35.0, 85.9
//   move.to(42.5, 85.9, 7, 12, YKP, TKP, 12, 6);
//   // face to 74.3, 100.6
//   move.to(74.3, 108.6, 7, 12, YKP, TKP, 12, 6);
//   //delay(700);

//   // old stacked rings
//   // face to 62.1, 128.5
//   move.to(66.1, 134.5, 7, 12, YKP, TKP, 12, 6);
//   // b face to 61.4, 95.4
//   move.bto(61.4, 110.4, 7, 12, YKP, TKP, 12, 6);
//   // face to 78.8, 128.0
//   delay(1000);
//   enableStopper = true;
//   holdRing();
//   move.to(82.8, 134.0, 7, 12, YKP, TKP, 12, 6);

//   // face to 77.6, 133.7
//   move.to(77.6, 135.7, 7, 12, YKP, TKP, 12, 0);
//   delay(800);

//   move.voldrive(6, 6);
//   delay(600);

//   move.swingOnLeft(270, TKP, TKI, TKD, 12, 5);
//   move.face(240, TKP, TKI, TKD, 12, 8);
//   BackMech.set(0);
//   delay(200);

//   move.voldrive(-7, -7);
//   delay(600);

//   // // new stacked rings
//   // arm.setStopping(hold);
//   // arm.spinFor(1800, degrees, false);
//   // // pps to 77.4, 105.6
//   // move.PPSto(80.4, 105.6, 7.5, 7.5, YKP, TKP, 12, 8/*BL*/, 12/*RAng*/, 6/*TE*/, 22/*Look*/);
//   // delay(600);
//   // intake.setVelocity(40, percent);

//   // // pps to 81.0, 146.9
//   // move.PPSto(81.0, 143.9, 12, 12, YKP, TKP, 12, 9/*BL*/, 17/*RAng*/, 6/*TE*/, 22/*Look*/);
//   // //intake.setVelocity(100, percent);

//   // // pps to 60.2, 150.7
//   // move.PPSto(60.2, 150.7, 7.5, 7.5, YKP, TKP, 12, 8/*BL*/, 12/*RAng*/, 6/*TE*/, 22/*Look*/);

//   // // b pps to 70.5, 152.5
//   // move.PPSbto(70.5, 152.5, 6, 6, YKP, TKP*1.1, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 16/*Look*/);

//   // // pps to 65.5, 136.8
//   // move.PPSto(65.5, 136.8, 7.5, 7.5, YKP, TKP, 12, 8/*BL*/, 12/*RAng*/, 6/*TE*/, 22/*Look*/);
//   // delay(400);
//   // intake.setVelocity(100, percent);
//   // delay(1200);
//   // BackMech.set(0);
//   // delay(200);
//   // move.face(225, TKP, TKI, TKD, 12, 3);
//   // move.voldrive(-4, -4);
//   // delay(1500);
//   // arm.setStopping(coast);
//   // arm.stop();

// // ALLIANCE STAKE
//   move.to(37.6, 120.6, 7, 12, YKP, TKP, 12, 9);
//   // b face to -4.4, 147.9
//   move.bto(-4.4, 147.9, 7, 12, YKP, TKP, 12, 6);
//   move.face(180, TKP, TKI, TKD, 12, 20);
//   move.straighth(-10, chassis.h, YKP, YKD, TKP, TKD, 4, 12, 12, 0);
//   delay(600);
//   enableStopper = false;
//   continueRing();
//   intake.setVelocity(100, percent);
//   intake.spin(forward);
//   delay(600);

// // FOURTH MOGO MOVE
//    // face to 37.6, 109.6
//   // move.to(37.6, 109.6, 7, 12, YKP, TKP, 12, 9);
//   // face to -12.6, 140.2
//   move.to(-12.6, 140.2, 7, 12, YKP, TKP, 12, 9);
//   // face to -42.5, 148.9
//   move.facePoint(-42.5, 148.9, TKP, TKI, TKD, 7, 6, false);
//   move.to(-42.5, 148.9, 7, 12, YKP, TKP, 12, 6);
 
//   // voldrive
//   move.straighth(100, chassis.h, YKP, YKD, TKP, TKD, 9.5, 12, 12, 0);
//   delay(1000);
//   move.voldrive(-4, -4);
  
  

// autonStop();
// }

// void WPRASkills(){
//   delay(7000);
//   Stick.set(0);
//   NeutralLock.set(0);
//   auto_settings();
//   starttime = Brain.timer(msec);
//   intake_lift = false;
//   chassis.setPos(0, 0, 0);
//   TKP = 1.1; //1.0 and 1.1 works good as well (but potential to undershoot) // 1.1 
//   YKP = 2.60; // was 2.4, was 2.65
//   move.cataEnabled = false;

//   enableNeutralStopper = false;
//   enableStopper = false;

//   delay(200);

//   intake.setVelocity(100, percent);
//   move.straighth(-10, chassis.h, YKP, YKD, TKP, TKD, 4, 12, 12, 0);
//   delay(600);
//   intake.spin(forward);
//   delay(700);
//   intake.stop();

//   // to 0.1, 8.9
//   move.to(0.1, 8.9, 7, 12, YKP, TKP, 12, 6);
//   delay(200);

//   // b face to (to mogo) 27.1, 8.5 
//   move.facePoint(30.1, 8.5, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(30.1, 8.5, 4, 12, YKP, TKP, 12, 9);
//   //delay(200);

//   BackMech.set(1);
//   move.voldrive(0, 0);
//   delay(400); // no swing

//   intake.setVelocity(100, percent);
//   intake.spin(forward);

//   // face to 32.8, 36.4
//   move.facePoint(32.8, 36.4, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(32.8, 36.4, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // // face to 48.3, 55.4
//   // move.facePoint(48.3, 55.4, TKP, TKI, TKD, 7, 6, false);
//   // //delay(200);
//   // move.to(48.3, 55.4, 7, 12, YKP, TKP, 12, 6);
//   // //delay(200);

//   // face to 81.4, 73.0
//   move.facePoint(81.4, 73.0, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(81.4, 73.0, 7, 12, YKP, TKP, 12, 6);
//   //delay(800);

//   // b to (no face) 68.5, 65.9
//   move.facePoint(68.5, 65.9, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(68.5, 65.9, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to 66.1, 43.2
//   move.facePoint(66.1, 43.2, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(66.1, 43.2, 7, 12, YKP, TKP, 12, 11); // bl was 6
//   //delay(200);

//   // face to 66.1, 13.9
//   move.facePoint(66.1, 13.9, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(66.1, 13.9, 7, 12, YKP, TKP, 12, 11); // bl was 6
//   //delay(200);

//   // face to 66.2, -4.7
//   move.facePoint(66.2, -4.7, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(66.2, -4.7, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // // b to (no face) 68.4, 35.2
//   // move.facePoint(68.4, 35.2, TKP, TKI, TKD, 7, 6, true);
//   // //delay(200);
//   // move.bto(68.4, 35.2, 7, 12, YKP, TKP, 12, 6);
//   // //delay(200);

//   // // face to 80.5, 11.5 
//   // move.facePoint(80.5, 11.5, TKP, TKI, TKD, 7, 6, false);
//   // //delay(200);
//   // move.to(80.5, 11.5, 7, 12, YKP, TKP, 12, 6);
//   // //delay(400);

//   // pps to 81.8, 12.0
//   move.PPSto(81.8, 12.0, 6, 6, YKP, TKP, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 22/*Look*/);

//   // // b to 56.2, 23.2
//   // move.bto(56.2, 23.2, 7, 12, YKP, TKP*0.8, 7, 6);

//   // // b face to 76.4, -6.3
//   // move.facePoint(76.4, -6.3, TKP, TKI, TKD, 7, 6, true);
//   // //delay(200);
//   // move.bto(76.4, -6.3, 12, 12, YKP, TKP, 7, 9);
//   // //delay(200);

//   // // mogo drop off
//   // move.face(315, TKP, TKI, TKD, 12, 8);
//   // BackMech.set(0);
//   // delay(200);
//   // intake.stop();
//   move.PPSbto(76.2, -1.5, 6, 6, YKP, TKP, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 22/*Look*/);
//   move.face(330, TKP, TKI, TKD, 12, 3);
//   BackMech.set(0);
//   move.voldrive(-4, -4);
//   delay(400);
//   move.voldrive(3, 3);
//   delay(400);

//   // SECOND MOGO /////////////////////////////////////////////////////////
//   // // to (no face) 63.8, 6.4
//   move.to(63.8, 1.2, 7, 12, YKP, TKP, 12, 6);
//   delay(200);

//   // // face to -6.0, 5.2
//   // // was 8.4 on y
//   // move.facePoint(-6.0, 7.4, TKP*1.2, TKI, TKD*1.3, 7, 6, false);
//   // delay(200);
//   // move.to(-6.0, 7.4, 12, 12, YKP, TKP*1.2, 12, 6);
//   // delay(200);

//   // // face b to -37.6, 6.3
//   // // was 8.4 on y
//   // move.facePoint(-37.6, 7.4, TKP, TKI, TKD*1.3, 7, 6, true);
//   // delay(200);
//   // //chassis.setPos(-2.0, 3.9, chassis.h); // RESET (x is offset to the right to push entire route to the left)
//   // move.bto(-37.6, 7.4, 4, 12, YKP, TKP, 12, 7);
//   // //delay(200);

//   move.PPSbto(-33.8, 7.1, 6, 6, YKP, TKP, 12, 3/*BL*/, 20/*RAng*/, 9/*TE*/, 22/*Look*/);
//   BackMech.set(1);
//   delay(200);


//   // clamp
//   BackMech.set(1);
//   move.voldrive(0, 0);
//   delay(400); // no swing
//   intake.setVelocity(100, percent);
//   intake.spin(forward);

//   // face to -35.4, 34.0
//   move.facePoint(-35.4, 34.0, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(-35.4, 34.0, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to -63.7, 37.3
//   move.facePoint(-63.7, 33.3, TKP*1.2, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(-63.7, 33.3, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to -81.7, 69.7
//   move.facePoint(-81.7, 69.7, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(-81.7, 69.7, 7, 12, YKP, TKP, 12, 6);
//   //delay(400);

//   // b to (no face) -64.5, 45.9 
//   move.facePoint(-64.5, 45.9, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(-64.5, 45.9, 7, 12, YKP, TKP, 12, 6);

//   // face to -68.4, 9.4
//   move.facePoint(-64.4, 6.4, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(-64.4, 6.4, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to -68.6, -6.6
//   move.facePoint(-64.6, -14.6, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(-64.6, -14.6, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // b to (no face) -67.8, 32.8
//   move.facePoint(-67.8, 32.8, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(-67.8, 32.8, 7, 12, YKP, TKP, 12, 6);

//   // face to -85.0, 8.0
//   move.facePoint(-80.5, 4.0, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(-80.5, 4.0, 7, 12, YKP, TKP, 12, 6);
//   //delay(1000);

//   // // b to (no face) -62.7, 20.1
//   // move.facePoint(-62.7, 20.1, TKP, TKI, TKD, 7, 6, true);
//   // //delay(200);
//   // move.bto(-62.7, 20.1, 7, 12, YKP, TKP, 12, 6);

//   // // b face to -81.4, -8.2
//   // move.facePoint(-79.4, -9.2, TKP, TKI, TKD, 7, 6, true);
//   // //delay(200);
//   // move.bto(-79.4, -9.2, 12, 12, YKP, TKP, 7, 9);
//   // //delay(200);

//   // // drop mogo off
//   // move.face(45, TKP, TKI, TKD, 12, 8);
//   // BackMech.set(0);
//   // delay(200);
//   // intake.stop();
//   move.PPSbto(-79.4, -2.7, 6, 6, YKP, TKP, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 22/*Look*/);
//   move.face(45, TKP, TKI, TKD, 12, 3);
//   BackMech.set(0);
//   move.voldrive(-4, -4);
//   delay(400);
//   intake.setVelocity(30, percent);
//   intake.spin(forward);
//   enableStopper = true;
//   holdRing();

//   // NEW ROUTE THIRD MOGO

//   // pps to -71.2, 100.6
//   //move.facePoint(64.7, 33.6, TKP, TKI, TKD, 12, 10, false); 
//   move.PPSto(-68.2, 100.6, 12, 12, YKP, TKP*0.6, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 22/*Look*/);

//   // face pps to -40.2, 102.5
//   move.facePoint(-40.2, 102.5, TKP, TKI, TKD, 7, 6, false);
//   move.PPSto(-40.2, 102.5, 12, 12, YKP, TKP*0.4, 12, 3/*BL*/, 5/*RAng*/, 6/*TE*/, 22/*Look*/);

//   // b pps to -7.5, 137.2
//   move.facePoint(-7.5, 137.2, TKP, TKI, TKD, 7, 6, true);
//   move.PPSbto(-7.5, 137.2, 6, 6, YKP, TKP*0.6, 12, 3/*BL*/, 5/*RAng*/, 6/*TE*/, 22/*Look*/);
//   BackMech.set(1);
//   delay(200);

//   autonStop();

//   // REDO THIRD MOGO /////////////////////////////////////////////////////
//   move.facePoint(-69.6, 5.0, TKP, TKI, TKD, 12, 6, false);
//   move.to(-69.6, 5.0, 12, 12, YKP, TKP*1.2, 12, 6);
//   delay(200);

//   // to -65.1, 75.5 
//   move.facePoint(-65.1, 75.5, TKP, TKI, TKD, 7, 6, false); 
//   move.to(-65.1, 75.5, 12, 12, YKP, TKP, 12, 6); 

//   // to -28.6, 107.0 (hold ring)
//   enableStopper = true;
//   holdRing();
//   move.facePoint(-28.6, 109.0, TKP, TKI, TKD, 7, 6, false); 
//   move.to(-28.6, 109.0, 7, 12, YKP, TKP*1.2, 12, 6); 
//   delay(300); // was 800

//   // b face to clamp -0.4, 130.5
//   move.facePoint(-0.4, 138.0, TKP*1.2, TKI, TKD, 7, 6, true);
//   delay(200);
//   move.bto(-0.4, 138.0, 4.2, 12, YKP, TKP*1.2, 12, 6);
//   BackMech.set(1);
//   move.voldrive(0, 0);
//   delay(400); // no swing
//   intake.setVelocity(100, percent);
//   enableStopper = false;
//   continueRing();

//   // face to -76.0, 94.9
//   move.facePoint(-79.0, 94.9, TKP*1.2, TKI, TKD, 7, 6, false);
//   intake.spin(forward);
//   move.to(-79.0, 94.9, 7, 12, YKP, TKP, 12, 6);
//   // b face to 4.3, 130.8
//   move.facePoint(4.3, 136.8, TKP, TKI, TKD, 7, 6, true);
//   move.bto(4.3, 136.8, 7, 12, YKP, TKP, 12, 6);
//   // face to 35.0, 85.9
//   move.facePoint(37.5, 85.9, TKP, TKI, TKD, 7, 6, false);
//   move.to(37.5, 85.9, 7, 12, YKP, TKP, 12, 6);
//   // face to 74.3, 100.6
//   move.facePoint(74.3, 108.6, TKP, TKI, TKD, 7, 6, false);
//   move.to(74.3, 108.6, 7, 12, YKP, TKP, 12, 6);
//   //delay(700);

//   // // face to 62.1, 128.5
//   // move.facePoint(66.1, 139.5, TKP, TKI, TKD, 7, 6, false);
//   // move.to(66.1, 139.5, 7, 12, YKP, TKP, 12, 6);
//   // // b face to 61.4, 95.4
//   // move.facePoint(61.4, 95.4, TKP, TKI, TKD, 7, 6, true);
//   // move.bto(61.4, 95.4, 7, 12, YKP, TKP, 12, 6);
//   // // face to 78.8, 128.0
//   // move.facePoint(82.8, 138.0, TKP, TKI, TKD, 7, 6, false);
//   // move.to(82.8, 138.0, 7, 12, YKP, TKP, 12, 6);

//   // // face to 77.6, 133.7
//   // move.facePoint(77.6, 138.7, TKP, TKI, TKD, 7, 6, false);
//   // move.to(77.6, 138.7, 7, 12, YKP, TKP, 12, 6);

//   // // stick clear
//   // //Stick.set(1);
//   // move.face(90, TKP, TKI, TKD, 12, 20);
//   // move.face(180, TKP, TKI, TKD, 12, 20);
//   // move.face(220, TKP, TKI, TKD, 12, 8);
//   // delay(400);
//   // //Stick.set(0);
//   // BackMech.set(0);
//   // move.straighth(-30, chassis.h, YKP, YKD, TKP, TKD, 8, 12, 12, 0);
//   // delay(900);
//   // //intake.spin(reverse);
//   // BackMech.set(0);
//   // //delay(200);
//   // intake.stop();

//   arm.setStopping(hold);
//   arm.spinFor(1800, degrees, false);
//   // pps to 77.4, 105.6
//   move.PPSto(80.4, 105.6, 7.5, 7.5, YKP, TKP, 12, 8/*BL*/, 12/*RAng*/, 6/*TE*/, 22/*Look*/);
//   delay(600);
//   intake.setVelocity(40, percent);

//   // pps to 81.0, 146.9
//   move.PPSto(81.0, 143.9, 12, 12, YKP, TKP, 12, 9/*BL*/, 17/*RAng*/, 6/*TE*/, 22/*Look*/);
//   //intake.setVelocity(100, percent);

//   // pps to 60.2, 150.7
//   move.PPSto(60.2, 150.7, 7.5, 7.5, YKP, TKP, 12, 8/*BL*/, 12/*RAng*/, 6/*TE*/, 22/*Look*/);

//   // b pps to 70.5, 152.5
//   move.PPSbto(70.5, 152.5, 6, 6, YKP, TKP*1.1, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 16/*Look*/);

//   // pps to 65.5, 136.8
//   move.PPSto(65.5, 136.8, 7.5, 7.5, YKP, TKP, 12, 8/*BL*/, 12/*RAng*/, 6/*TE*/, 22/*Look*/);
//   delay(400);
//   intake.setVelocity(100, percent);
//   delay(1200);
//   BackMech.set(0);
//   delay(200);
//   move.face(225, TKP, TKI, TKD, 12, 3);
//   move.voldrive(-4, -4);
//   delay(1500);
//   arm.setStopping(coast);
//   arm.stop();

//   // face to 37.6, 109.6
//   move.facePoint(37.6, 109.6, TKP, TKI, TKD, 7, 6, false);
//   move.to(37.6, 109.6, 7, 12, YKP, TKP, 12, 9);
//   // face to -12.6, 140.2
//   move.facePoint(-12.6, 140.2, TKP, TKI, TKD, 7, 6, false);
//   move.to(-12.6, 140.2, 7, 12, YKP, TKP, 12, 9);
//   // face to -42.5, 148.9
//   move.facePoint(-42.5, 148.9, TKP, TKI, TKD, 7, 6, false);
//   move.to(-42.5, 148.9, 7, 12, YKP, TKP, 12, 6);
 
//   // voldrive
//   move.straighth(100, chassis.h, YKP, YKD, TKP, TKD, 9.5, 12, 12, 0);
//   delay(1500);
//   move.voldrive(-4, -4);

//   autonStop(); /////////////////////////////////////////////////////////////////////////////////////

//   // THIRD MOGO //////////////////////////////////////////////////////////////////////
//   // to (no face) -66.6, 5.0
//   move.facePoint(-69.6, 5.0, TKP, TKI, TKD, 12, 6, false);
//   move.to(-69.6, 5.0, 12, 12, YKP, TKP*1.2, 12, 6);
//   delay(200);

//   // face to -66.6, 69.3
//   move.facePoint(-63.6, 69.3, TKP, TKI, TKD, 7, 6, false); 
//   move.to(-63.6, 69.3, 12, 12, YKP, TKP, 12, 6); 

//   // face to (hold ring) -33.6, 94.2
//   enableStopper = true;
//   holdRing();
//   move.facePoint(-34.6, 102.2, TKP, TKI, TKD, 7, 6, false); // higher and farther, was -37.6, 97.2
//   move.to(-34.6, 102.2, 7, 12, YKP, TKP*1.2, 12, 6); // higher and farther, was -37.6, 97.2
//   delay(300); // was 800

//   // face b to 7.0, 133.3
//   move.facePoint(7.0, 136.3, TKP*1.2, TKI, TKD, 7, 6, true); // higher, was 7.0, 133.3
//   delay(200);
//   move.bto(7.0, 136.3, 6.2, 12, YKP, TKP*1.2, 12, 6); // speed was 4 (consistent but too slow)

//   move.voldrive(-4, -4);
//   delay(400);

//   // clamp (disable stopper)
//   BackMech.set(1);
//   move.voldrive(0, 0);
//   delay(400); // no swing
//   intake.setVelocity(100, percent);
//   enableStopper = false;
//   continueRing();

//   // face to -62.9, 97.9
//   move.facePoint(-67.9, 91.9, TKP*1.2, TKI, TKD, 7, 6, false);
//   intake.spin(forward);
//   move.to(-67.9, 91.9, 7, 12, YKP, TKP, 12, 6);

//   // face b to 9.8, 129.8
//   move.facePoint(9.8, 129.8, TKP, TKI, TKD, 7, 6, true);
//   move.bto(9.8, 129.8, 7, 12, YKP, TKP, 12, 6);

//   // face to 33.5, 95.5 
//   move.facePoint(34.5, 95.5, TKP, TKI, TKD, 7, 6, false);
//   move.to(34.5, 95.5, 7, 12, YKP, TKP, 12, 6);

//   // face to 65.7, 94.3
//   move.facePoint(65.7, 102.8, TKP, TKI, TKD, 7, 6, false);
//   move.to(65.7, 102.8, 7, 12, YKP, TKP, 12, 6);
//   delay(700);

//   // face to 74.5, 105.5
//   move.facePoint(78.5, 105.5, TKP, TKI, TKD, 7, 6, false);
//   //Stick.set(1);
//   move.to(78.5, 105.5, 7, 12, YKP, TKP, 12, 6);
//   //move.face(0, TKP, TKI, TKD, 12, 8);

//   // face to 74.8, 136.3
//   move.facePoint(81.8, 139.3, TKP, TKI, TKD, 7, 6, false);
//   Stick.set(1);
//   move.to(81.8, 139.3, 7, 12, YKP, TKP, 12, 6);

//   move.face(90, TKP, TKI, TKD, 12, 8);
//   move.face(180, TKP, TKI, TKD, 12, 8);
//   //move.face(180, TKP, TKI, TKD, 12, 8);
//   move.face(220, TKP, TKI, TKD, 12, 8);
//   delay(200);
//   Stick.set(0);
//   move.straighth(-15, chassis.h, YKP, YKD, TKP, TKD, 8, 12, 12, 0);
//   delay(600);
//   //intake.spin(reverse);
//   BackMech.set(0);
//   //delay(200);
//   intake.stop();

//   autonStop();

//   // Max speed curve 20.0, 116.1 and -10.7, 137.7
//   move.CurveTo(20.0, 125.1, -10.7, 160.7, 12, 12, YKP, TKP, 12, 18, 14); // lookahead was 14

//   // Max speed curve 53.5, 101.7 and -6.8, 141.1
//   //move.CurveTo(53.5, 101.7, -6.8, 141.1, 12, 12, YKP, TKP, 12, 20, 5); // lookahead was 14

//   // straight ram at 290
//   move.face(290, TKP, TKI, TKD, 12, 10);
//   move.straighth(100, chassis.h, YKP, YKD, TKP, TKD, 9.5, 12, 12, 0);
//   delay(1500);
//   move.voldrive(-4, -4);



//   autonStop();
// }


// void FinalMOASkills(){
//   //delay(7000);
//   NeutralLock.set(0);
//   auto_settings();
//   starttime = Brain.timer(msec);
//   intake_lift = false;
//   chassis.setPos(0, 0, 0);
//   TKP = 1.1; //1.0 and 1.1 works good as well (but potential to undershoot) // 1.1 
//   YKP = 2.4;
//   move.cataEnabled = false;

//   enableNeutralStopper = false;
//   enableStopper = false;

//   delay(200);

//   intake.setVelocity(100, percent);
//   move.straighth(-10, chassis.h, YKP, YKD, TKP, TKD, 4, 12, 12, 0);
//   delay(600);
//   intake.spin(forward);
//   delay(400);
//   intake.stop();

//   // to 0.1, 8.9
//   move.to(0.1, 8.9, 7, 12, YKP, TKP, 12, 6);
//   delay(200);

//   // b face to (to mogo) 27.1, 8.5 
//   move.facePoint(30.1, 8.5, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(30.1, 8.5, 4, 12, YKP, TKP, 12, 9);
//   //delay(200);

//   BackMech.set(1);
//   move.voldrive(0, 0);
//   delay(400); // no swing

//   intake.setVelocity(100, percent);
//   intake.spin(forward);

//   // face to 32.8, 36.4
//   move.facePoint(32.8, 36.4, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(32.8, 36.4, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to 48.3, 55.4
//   move.facePoint(48.3, 55.4, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(48.3, 55.4, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to 81.4, 73.0
//   move.facePoint(81.4, 73.0, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(81.4, 73.0, 7, 12, YKP, TKP, 12, 6);
//   //delay(800);

//   // b to (no face) 68.5, 65.9
//   move.facePoint(68.5, 65.9, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(68.5, 65.9, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to 66.1, 43.2
//   move.facePoint(66.1, 43.2, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(66.1, 43.2, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to 66.1, 13.9
//   move.facePoint(66.1, 13.9, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(66.1, 13.9, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to 66.2, -4.7
//   move.facePoint(66.2, -4.7, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(66.2, -4.7, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // b to (no face) 68.4, 35.2
//   move.facePoint(68.4, 35.2, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(68.4, 35.2, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to 80.5, 11.5 
//   move.facePoint(80.5, 11.5, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(80.5, 11.5, 7, 12, YKP, TKP, 12, 6);
//   //delay(400);

//   // b to 56.2, 23.2
//   move.bto(56.2, 23.2, 7, 12, YKP, TKP*0.8, 7, 6);

//   // b face to 76.4, -6.3
//   move.facePoint(76.4, -6.3, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(76.4, -6.3, 12, 12, YKP, TKP, 7, 9);
//   //delay(200);

//   // mogo drop off
//   move.face(315, TKP, TKI, TKD, 12, 8);
//   BackMech.set(0);
//   delay(200);
//   intake.stop();

//   // SECOND MOGO /////////////////////////////////////////////////////////
//   // to (no face) 63.8, 6.4
//   move.to(63.8, 5.2, 7, 12, YKP, TKP, 12, 6);
//   delay(200);

//   // face to -6.0, 5.2
//   move.facePoint(-6.0, 8.4, TKP*1.2, TKI, TKD*1.3, 7, 6, false);
//   delay(200);
//   move.to(-6.0, 8.4, 12, 12, YKP, TKP*1.2, 12, 6);
//   delay(200);

//   // face b to -37.6, 6.3
//   move.facePoint(-37.6, 8.4, TKP, TKI, TKD*1.3, 7, 6, true);
//   delay(200);
//   //chassis.setPos(-2.0, 3.9, chassis.h); // RESET (x is offset to the right to push entire route to the left)
//   move.bto(-37.6, 8.4, 4, 12, YKP, TKP, 12, 7);
//   //delay(200);

//   // clamp
//   BackMech.set(1);
//   move.voldrive(0, 0);
//   delay(400); // no swing
//   intake.setVelocity(100, percent);
//   intake.spin(forward);

//   // face to -35.4, 34.0
//   move.facePoint(-35.4, 34.0, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(-35.4, 34.0, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to -63.7, 37.3
//   move.facePoint(-63.7, 33.3, TKP*1.2, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(-63.7, 33.3, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to -81.7, 69.7
//   move.facePoint(-81.7, 69.7, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(-81.7, 69.7, 7, 12, YKP, TKP, 12, 6);
//   //delay(400);

//   // b to (no face) -64.5, 45.9 
//   move.facePoint(-64.5, 45.9, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(-64.5, 45.9, 7, 12, YKP, TKP, 12, 6);

//   // face to -68.4, 9.4
//   move.facePoint(-64.4, 6.4, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(-64.4, 6.4, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // face to -68.6, -6.6
//   move.facePoint(-64.6, -12.6, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(-64.6, -12.6, 7, 12, YKP, TKP, 12, 6);
//   //delay(200);

//   // b to (no face) -67.8, 32.8
//   move.facePoint(-67.8, 32.8, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(-67.8, 32.8, 7, 12, YKP, TKP, 12, 6);

//   // face to -85.0, 8.0
//   move.facePoint(-80.5, 8.0, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(-80.5, 8.0, 7, 12, YKP, TKP, 12, 6);
//   //delay(1000);

//   // b to (no face) -62.7, 20.1
//   move.facePoint(-62.7, 20.1, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(-62.7, 20.1, 7, 12, YKP, TKP, 12, 6);

//   // b face to -81.4, -8.2
//   move.facePoint(-78.4, -8.2, TKP, TKI, TKD, 7, 6, true);
//   //delay(200);
//   move.bto(-78.4, -8.2, 12, 12, YKP, TKP, 7, 9);
//   //delay(200);

//   // drop mogo off
//   move.face(45, TKP, TKI, TKD, 12, 8);
//   BackMech.set(0);
//   delay(200);
//   intake.stop();

//   // THIRD MOGO //////////////////////////////////////////////////////////////////////
//   // to (no face) -66.6, 5.0
//   move.facePoint(-69.6, 5.0, TKP, TKI, TKD, 12, 6, false);
//   move.to(-69.6, 5.0, 12, 12, YKP, TKP*1.2, 12, 6);
//   delay(200);

//   // face to -66.6, 69.3
//   move.facePoint(-63.6, 69.3, TKP, TKI, TKD, 7, 6, false);
//   move.to(-63.6, 69.3, 12, 12, YKP, TKP, 12, 6);

//   // face to (hold ring) -33.6, 94.2
//   enableStopper = true;
//   holdRing();
//   move.facePoint(-37.6, 97.2, TKP, TKI, TKD, 7, 6, false);
//   move.to(-37.6, 97.2, 7, 12, YKP, TKP*1.2, 12, 6);
//   delay(300); // was 800

//   // face b to 7.0, 133.3
//   move.facePoint(7.0, 138.3, TKP*1.2, TKI, TKD, 7, 6, true);
//   delay(200);
//   move.bto(7.0, 138.3, 6.2, 12, YKP, TKP*1.2, 12, 6); // speed was 4 (consistent but too slow)

//   // clamp (disable stopper)
//   BackMech.set(1);
//   move.voldrive(0, 0);
//   delay(400); // no swing
//   intake.setVelocity(100, percent);
//   enableStopper = false;
//   continueRing();

//   // face to -62.9, 97.9
//   move.facePoint(-68.9, 91.9, TKP*1.2, TKI, TKD, 7, 6, false);
//   intake.spin(forward);
//   move.to(-68.9, 91.9, 7, 12, YKP, TKP, 12, 6);

//   // face b to 9.8, 129.8
//   move.facePoint(9.8, 129.8, TKP, TKI, TKD, 7, 6, true);
//   move.bto(9.8, 129.8, 7, 12, YKP, TKP, 12, 6);

//   // face to 33.5, 95.5 
//   move.facePoint(34.5, 95.5, TKP, TKI, TKD, 7, 6, false);
//   move.to(34.5, 95.5, 7, 12, YKP, TKP, 12, 6);

//   // face to 65.7, 94.3
//   move.facePoint(65.7, 100.8, TKP, TKI, TKD, 7, 6, false);
//   move.to(65.7, 100.8, 7, 12, YKP, TKP, 12, 6);
//   delay(700);

//   // face to 74.5, 105.5
//   move.facePoint(78.5, 105.5, TKP, TKI, TKD, 7, 6, false);
//   //Stick.set(1);
//   move.to(78.5, 105.5, 7, 12, YKP, TKP, 12, 6);
//   //move.face(0, TKP, TKI, TKD, 12, 8);

//   // face to 74.8, 136.3
//   move.facePoint(79.8, 136.3, TKP, TKI, TKD, 7, 6, false);
//   move.to(79.8, 136.3, 7, 12, YKP, TKP, 12, 6);

//   move.face(270, TKP, TKI, TKD, 12, 8);
//   //move.face(180, TKP, TKI, TKD, 12, 8);
//   move.face(220, TKP, TKI, TKD, 12, 8);
//   delay(200);
//   move.straighth(-15, chassis.h, YKP, YKD, TKP, TKD, 8, 12, 12, 0);
//   delay(600);
//   //intake.spin(reverse);
//   BackMech.set(0);
//   //delay(200);
//   intake.stop();

//   // Max speed curve 20.0, 116.1 and -10.7, 137.7
//   move.CurveTo(20.0, 125.1, -10.7, 160.7, 12, 12, YKP, TKP, 12, 18, 14); // lookahead was 14

//   // Max speed curve 53.5, 101.7 and -6.8, 141.1
//   //move.CurveTo(53.5, 101.7, -6.8, 141.1, 12, 12, YKP, TKP, 12, 20, 5); // lookahead was 14

//   // straight ram at 290
//   move.face(290, TKP, TKI, TKD, 12, 10);
//   move.straighth(100, chassis.h, YKP, YKD, TKP, TKD, 9.5, 12, 12, 0);
//   delay(1500);
//   move.voldrive(-4, -4);

//   autonStop();

// }

// ///////////////////////////////////////////////////////
// int Flip; // **1 for RED, -1 for BLUE**
// ///////////////////////////////////////////////////////

// void AWP(){
//   //delay(7000);
//   Flip = -1; // BLUE //////////////////////////////////// REMEMBER TO CHANGE FOR QUAL
//   NeutralLock.set(0);
//   auto_settings();
//   starttime = Brain.timer(msec);
//   intake_lift = false;
//   chassis.setPos(0, 0, 270*Flip);
//   TKP = 1.1;
//   move.cataEnabled = false;
//   enableNeutralStopper = false;
//   enableStopper = false;
//   delay(200);
//   intake.setVelocity(100, percent);

// // SECTION 1 (alliance) ////////////////////////////////////////////////////////
//   move.straighth(-18.3, 270*Flip, YKP, YKD*1.2, TKP*1.3, TKD, 12, 12, 12, 2);
//   move.straighth(1.1, 270*Flip, YKP, YKD*1.2, TKP*1.3, TKD, 12, 12, 12, 0.6);
//   move.face(0, TKP, TKI, TKD*1.3, 12, 1);
//   delay(400); // settle
//   move.straighth(-12, 0, YKP, YKD*.8, TKP*1.3, TKD, 6, 12, 12, 0);
//   delay(650);
//   intake.spin(forward);
//   move.voldrive(-2, -2);
//   delay(500);
//   move.straighth(8, 0, YKP, YKD*1.2, TKP*1.3, TKD, 5, 12, 12, 2);
//   intake.stop();
// ////////////////////////////////////////////////////////////////////////////////

// // SECTION 2 (goal + 3 rings) //////////////////////////////////////////////////
//   move.facePoint(-4.0*Flip, 32.6, TKP, TKI, TKD*1.3, 12, 4, true);
//   move.bto(-4.0*Flip, 32.6, 12, 12, YKP, TKP*1.3, 12, 4.2);
//   move.voldrive(-4, -4);
//   delay(400);
//   BackMech.set(1);
//   move.veldrive(5, 5);
//   delay(30);
//   move.voldrive(0, 0);
//   delay(200);

//   // face to -23.5, 59.9 and curve -58.0, 64.3
//   // was -26.5
//   intake.setVelocity(50, percent);
//   intake.spin(forward);
//   arm.setStopping(hold);
//   arm.setVelocity(100, percent);
//   arm.spinFor(1300, degrees, false);
//   move.facePoint(-34.5*Flip, 64.9, TKP*1.2, TKI, TKD*1.3, 8, 6, false);
//   move.to(-34.5*Flip, 64.9, 7, 12, YKP, TKP*1.2, 12, 6);
//   move.face(270*Flip, TKP, TKI, TKD*1.3, 12, 3);
//   //delay(400); // settle
//   move.straighth(24, 270*Flip, YKP, YKD*.8, TKP*1.3, TKD, 6, 12, 12, 6);
//   //move.CurveTo(-26.5*Flip, 60.9, -58.0*Flip, 58.3, 12, 12, YKP, TKP, 12, 10, 7);

//   // copy paste
//   move.facePoint(-46.2*Flip, 45.8, TKP*1.2, TKI, TKD*1.3, 8, 6, false);
//   arm.setStopping(coast);
//   arm.stop();
//   intake.setVelocity(100, percent);
//   move.to(-46.2*Flip, 45.8, 7, 12, YKP, TKP*1.2, 12, 6);
//   intake.spin(forward);
//   move.voldrive(-2, -2);
//   delay(250);
//   move.voldrive(0, 0);
//   delay(1000);
//   // intake.stop();

//   // face to 8.8, 42.1
//   move.facePoint(8.8*Flip, 42.1, TKP*1.2, TKI, TKD*1.3, 8, 6, false);
//   move.to(8.8*Flip, 42.1, 7, 12, YKP, TKP*1.2, 12, 6);
//   move.voldrive(6, 6);


//   autonStop();
//   ////////////////////////////////////////////////////////////////////////////////
// }

// ///////////////////////////////////////////////////////
// int ElimFlip; // **1 for RED, -1 for BLUE**
// ///////////////////////////////////////////////////////

// void Elim(){
//   //delay(7000);
//   ElimFlip = 1; // RED //////////////////////////////////// REMEMBER TO CHANGE FOR ELIM
//   NeutralLock.set(0);
//   auto_settings();
//   starttime = Brain.timer(msec);
//   intake_lift = false;
//   chassis.setPos(0, 0, 0*ElimFlip); 
//   TKP = 1.1;
//   move.cataEnabled = false;
//   enableNeutralStopper = false;
//   enableStopper = false;
//   delay(200);
//   intake.setVelocity(100, percent);

//   // SECTION 1 (Mogo) ////////////////////////////////////////////////////////
//   // face to 20.7, 38.7 
//   enableStopper = true;
//   holdRing();
//   move.facePoint(20.7*ElimFlip, 38.7, TKP*1.2, TKI, TKD*1.3, 8, 6, false);
//   move.to(20.7*ElimFlip, 38.7, 8, 12, YKP, TKP*1.2, 12, 6);

//   // face to 26.3, 55.8
//   move.facePoint(26.3*ElimFlip, 51.8, TKP*1.2, TKI, TKD*1.3, 8, 6, false);
//   move.to(26.3*ElimFlip, 51.8, 8, 12, YKP, TKP*1.2, 12, 3);
//   move.face(-10/**ElimFlip*/, TKP, TKI, TKD*1.3, 12, 3);
//   Stick.set(1);
//   delay(300);
//   move.face(90*ElimFlip, TKP, TKI, TKD*1.3, 8, 8);  // RED
//   move.face(195*ElimFlip, TKP, TKI, TKD*1.3, 5, 3); // RED
//   // move.face(90, TKP, TKI, TKD*1.3, 8, 8);  // BLUE
//   // move.face(195*ElimFlip, TKP, TKI, TKD*1.3, 5, 3); // BLUE
//   Stick.set(0);
//   delay(500);
//   move.face(0*ElimFlip, TKP, TKI, TKD*1.3, 12, 3);
//   move.voldrive(-6, -6);
//   delay(800);
//   BackMech.set(1);
//   move.veldrive(5, 5);
//   delay(200);
//   intake.setVelocity(100, percent);
//   intake.spinFor(300, degrees, true);
//   move.face(-22*ElimFlip, TKP, TKI, TKD*1.3, 12, 3);
//   move.voldrive(-4, -4);
//   delay(600);
//   BackMech.set(0);
//   move.voldrive(0, 0);
//   delay(200);
//   intake.spin(forward);

//   // face to 32.2, 43.7
//   move.facePoint(32.2*ElimFlip, 31.7, TKP*1.2, TKI, TKD*1.3, 8, 6, false);
//   move.to(32.2*ElimFlip, 31.7, 8, 12, YKP, TKP*1.2, 12, 3);
//   //delay(200);

//   // b face to -6.4, 41.9 and clamp
//   move.facePoint(-6.4*ElimFlip, 41.9, TKP, TKI, TKD*1.3, 12, 4, true);
//   move.bto(-6.4*ElimFlip, 41.9, 7, 12, YKP, TKP*1.3, 12, 3);
//   move.voldrive(-2, -2);
//   delay(200);
//   BackMech.set(1);
//   move.veldrive(5, 5);
//   delay(30);
//   move.voldrive(0, 0);
//   delay(200);

//   // face to -13.0, -4.8
//   move.facePoint(-13.0*ElimFlip, -1.8, TKP*1.2, TKI, TKD*1.3, 8, 6, false);
//   delay(200);
//   move.to(-13.0*ElimFlip, -1.8, 8, 12, YKP, TKP*1.2, 12, 3);

//   // b face to 14.9, -14.0
//   // move.facePoint(14.9*ElimFlip, -9.0, TKP, TKI, TKD*1.3, 12, 4, true);
//   // move.bto(14.9*ElimFlip, -9.0, 7, 12, YKP, TKP*1.3, 12, 8);
//   move.face(-95*ElimFlip, TKP, TKI, TKD*1.3, 12, 3);
//   move.voldrive(-2, -2);
//   delay(600);

//   autonStop();



// autonStop();
// }


// void Rush(){
//   //delay(7000);
//   Flip = 1; // RED ///////////////////////////////////////////////////////////////
//   NeutralLock.set(0);
//   auto_settings();
//   starttime = Brain.timer(msec);
//   intake_lift = false;
//   chassis.setPos(-4.0, 0, 180); // switch pos by slide
//   TKP = 1.1;
//   move.cataEnabled = false;
//   enableNeutralStopper = false;
//   enableStopper = false;
//   delay(200);
//   intake.setVelocity(100, percent);

//   // SECTION 1 (grab mogo) ////////////////////////////////////////////////////////
//   move.facePoint(-4.0*Flip, 32.6, TKP, TKI, TKD*1.3, 12, 4, true);
//   move.bto(-4.0*Flip, 32.6, 12, 12, YKP, TKP*1.3, 12, 3);
//   move.voldrive(-4, -4);
//   delay(400);
//   BackMech.set(1);
//   move.veldrive(5, 5);
//   delay(30);
//   move.voldrive(0, 0);
//   delay(200);
//   ////////////////////////////////////////////////////////////////////////////////

//   // SECTION 2 (middle rings) ////////////////////////////////////////////////////
//   move.facePoint(-38.3*Flip, 60.9, TKP*1.2, TKI, TKD*1.3, 8, 6, false);
//   intake.spin(forward);
//   move.to(-38.3*Flip, 60.9, 7, 12, YKP, TKP*1.2, 12, 6);
//   delay(200);
//   move.facePoint(-37.4*Flip, 47.4, TKP*1.2, TKI, TKD*1.3, 8, 6, true);
//   move.bto(-37.4*Flip, 47.4, 9, 12, YKP, TKP*1.4, 12, 6);
//   move.facePoint(-49.5*Flip, 60.4, TKP*1.2, TKI, TKD*1.3, 8, 6, false);
//   move.to(-49.5*Flip, 60.4, 7, 12, YKP, TKP*1.2, 12, 6);
//   delay(200);
//   ////////////////////////////////////////////////////////////////////////////////

//   // SECTION 3 (side ring) ///////////////////////////////////////////////////////
//   move.facePoint(-29.2*Flip, 45.8, TKP*1.2, TKI, TKD*1.3, 8, 6, false);
//   move.to(-29.2*Flip, 45.8, 7, 12, YKP, TKP*1.2, 12, 6);
//   move.face(260*Flip, TKP, TKI, TKD*1.3, 12, 1);
//   move.straighth(12, 260*Flip, YKP, YKD*.8, TKP*1.3, TKD, 6, 12, 12, 2);
//   move.voldrive(-2, -2);
//   delay(250);
//   /////////////////////////////////////////////////////////////////////////////////

// autonStop();
// }






// OLD SUPER CONSISTENT AWP
// ///////////////////////////////////////////////////////
// int Flip; // **1 for RED, -1 for BLUE**
// ///////////////////////////////////////////////////////

// void AWP(){
//   //delay(7000);
//   Flip = 1; // RED //////////////////////////////////// REMEMBER TO CHANGE FOR QUAL
//   NeutralLock.set(0);
//   auto_settings();
//   starttime = Brain.timer(msec);
//   intake_lift = false;
//   chassis.setPos(0, 0, 270*Flip);
//   TKP = 1.1;
//   move.cataEnabled = false;
//   enableNeutralStopper = false;
//   enableStopper = false;
//   delay(200);
//   intake.setVelocity(100, percent);

// // SECTION 1 (alliance) ////////////////////////////////////////////////////////
//   move.straighth(-18.3, 270*Flip, YKP, YKD*1.2, TKP*1.3, TKD, 12, 12, 12, 2);
//   move.straighth(1.1, 270*Flip, YKP, YKD*1.2, TKP*1.3, TKD, 12, 12, 12, 0.6);
//   move.face(0, TKP, TKI, TKD*1.3, 12, 1);
//   delay(400); // settle
//   move.straighth(-12, 0, YKP, YKD*.8, TKP*1.3, TKD, 6, 12, 12, 0);
//   delay(650);
//   intake.spin(forward);
//   move.voldrive(-2, -2);
//   delay(500);
//   move.straighth(8, 0, YKP, YKD*1.2, TKP*1.3, TKD, 5, 12, 12, 2);
//   intake.stop();
// ////////////////////////////////////////////////////////////////////////////////

// // SECTION 2 (goal + 3 rings) //////////////////////////////////////////////////
//   move.facePoint(-4.0*Flip, 32.6, TKP, TKI, TKD*1.3, 12, 4, true);
//   move.bto(-4.0*Flip, 32.6, 12, 12, YKP, TKP*1.3, 12, 4.2);
//   move.voldrive(-4, -4);
//   delay(400);
//   BackMech.set(1);
//   move.veldrive(5, 5);
//   delay(30);
//   move.voldrive(0, 0);
//   delay(200);
//   move.facePoint(-29.2*Flip, 45.8, TKP*1.2, TKI, TKD*1.3, 8, 6, false);
//   move.to(-29.2*Flip, 45.8, 7, 12, YKP, TKP*1.2, 12, 6);
//   intake.spin(forward);
//   move.face(260*Flip, TKP, TKI, TKD*1.3, 12, 1);
//   move.straighth(12, 260*Flip, YKP, YKD*.8, TKP*1.3, TKD, 6, 12, 12, 2);
//   intake.spin(forward);
//   move.voldrive(-2, -2);
//   delay(250);
//   move.facePoint(-38.3*Flip, 60.9, TKP*1.2, TKI, TKD*1.3, 8, 6, false);
//   move.to(-38.3*Flip, 60.9, 7, 12, YKP, TKP*1.2, 12, 6);
//   delay(200);
//   move.facePoint(-37.4*Flip, 47.4, TKP*1.2, TKI, TKD*1.3, 8, 6, true);
//   move.bto(-37.4*Flip, 47.4, 9, 12, YKP, TKP*1.4, 12, 6);
//   move.facePoint(-45.5*Flip, 62.4, TKP*1.2, TKI, TKD*1.3, 8, 6, false);
//   move.to(-45.5*Flip, 62.4, 7, 12, YKP, TKP*1.2, 12, 6);
//   delay(200);
//   ////////////////////////////////////////////////////////////////////////////////

//   // SECTION 3 (drop + bar touch) ////////////////////////////////////////////////
//   move.facePoint(-12.0*Flip, 23.8, TKP, TKI, TKD, 8, 6, true);
//   move.bto(-12.0*Flip, 23.8, 7, 12, YKP, TKP, 12, 6);
//   // BackMech.set(0); // SWITCHHH
//   // autonStop(); ////////////////////////////////////////// SWITCHHH
//   arm.setStopping(hold);
//   arm.setVelocity(100, percent);
//   arm.spinFor(2400, degrees, false);
//   move.face(0, TKP, TKI, TKD, 6, 6);
//   BackMech.set(0);
//   intake.stop();
//   delay(500); // delay(200);
//   move.facePoint(6*Flip, 53, TKP, TKI, TKD, 8, 6, false);
//   //move.to(6*Flip, 53, 7, 12, YKP, TKP, 12, 6);
//   move.voldrive(8, 8);
//   arm.stop(coast);

//   autonStop();
//   ////////////////////////////////////////////////////////////////////////////////
// }

// old elim code
// ///////////////////////////////////////////////////////
// int ElimFlip; // **1 for RED, -1 for BLUE**
// ///////////////////////////////////////////////////////

// void Elim(){
//   //delay(7000);
//   Flip = 1; // RED //////////////////////////////////// REMEMBER TO CHANGE FOR ELIM
//   NeutralLock.set(0);
//   auto_settings();
//   starttime = Brain.timer(msec);
//   intake_lift = false;
//   chassis.setPos(0, 0, 270*Flip); // heading 0 is to the left (field orientation)
//   TKP = 1.1;
//   move.cataEnabled = false;
//   enableNeutralStopper = false;
//   enableStopper = false;
//   delay(200);
//   intake.setVelocity(100, percent);

//   // SECTION 1 (Mogo) ////////////////////////////////////////////////////////
//   // b face to 45.3, -8.7
//   move.facePoint(45.3*Flip, -8.7, TKP, TKI, TKD*1.3, 6, 4, true);
//   move.bto(45.3*Flip, -8.7, 6, 12, YKP, TKP*1.3, 12, 4.2);
//   move.voldrive(-4, -4);
//   delay(300);
//   // clamp, wait, then intake spin, then wait, then face then drop
//   BackMech.set(1);
//   delay(200);
//   move.voldrive(5, 5);
//   delay(300);
//   intake.spin(forward);
//   move.voldrive(0, 0);
//   delay(400);
//   move.face(90*Flip, TKP, TKI, TKD, 6, 6);
//   move.voldrive(-8, -8);
//   delay(450);
//   BackMech.set(0);
//   intake.stop();

//   // face to 45.3, -35.1
//   move.facePoint(45.3*Flip, -35.1, TKP, TKI, TKD*1.3, 6, 4, false);
//   enableStopper = true;
//   holdRing();
//   move.to(45.3*Flip, -35.1, 6, 12, YKP, TKP*1.3, 6, 6.5); //bl was 4.2
//   delay(200);

//   move.facePoint(45.3*Flip, -38.1, TKP, TKI, TKD, 6, 4, false);
//   move.to(45.3*Flip, -38.1, 6, 12, YKP, TKP*1.3, 6, 2.5);

//   move.face(270*Flip, TKP, TKI, TKD, 6, 6);
//   //autonStop(); //////////TEMPORARY
//   move.straighth(-10, 270*Flip, YKP, YKD*.8, TKP*1.3, TKD, 4, 12, 12, 2);

//   // b face to (SLOW) 64.3, -38.0
//   // move.facePoint(64.3*Flip, -38.0, TKP, TKI, TKD*1.3, 6, 4, true);
//   // // move.bto(64.3*Flip, -38.0, 3, 12, YKP, TKP*1.3, 12, 4.2);
//   // move.straighthToPoint(-24, 64.3*Flip, -38.0, YKP, YKD, TKP, TKD, 4, 12, 12, 4.2, true);
//   // move.voldrive(-2, -2);
//   // delay(200);
//   // clamp, wait, then intake spin, then wait, then face 270 then drop
//   BackMech.set(1);
//   delay(200);
//   move.voldrive(5, 5);
//   delay(300);
//   intake.spin(forward);
//   move.voldrive(0, 0);
//   delay(400);
//   enableStopper = false;
//   continueRing();
//   intake.spin(forward);
//   ////////////////////////////////////////////////////////////////////////////////

//   // face to 3.8, -55.9
//   move.facePoint(1.8*Flip, -58.9, TKP, TKI, TKD*1.3, 6, 4, false);
//   delay(300);
//   intake.stop();
//   //Stick.set(1);
//   move.to(1.8*Flip, -58.9, 6, 12, YKP, TKP*1.3, 12, 4.2);

//   autonStop();

//   delay(200);
//   move.face(0*Flip, TKP, TKI, TKD, 6, 6);
//   Stick.set(0);
//   move.face(71*Flip, TKP, TKI, TKD, 6, 6);
//   move.voldrive(-4, -4);
//   delay(400);



// autonStop();
// }


// void NewMOASkills(){
//   delay(7000);
//   NeutralLock.set(0);
//   auto_settings();
//   starttime = Brain.timer(msec);
//   intake_lift = false;
//   chassis.setPos(0, 0, 270);

//   move.cataEnabled = false;

//   enableNeutralStopper = false;
//   enableStopper = false;

//   intake.setVelocity(100, percent);

//   NeutralLock.set(1);
//   delay(100);
//   arm.setStopping(hold);
//   arm.setVelocity(100, percent);
//   arm.spinFor(forward, 2200, degrees, true);

//   move.swingOnLeft(155, TKP, TKI, TKD, 6, 6);

//   move.straighth(-2, chassis.h, YKP, YKD, TKP, TKD, 6, 12, 12, 0.5);

//   arm.spinFor(reverse, 1200, degrees, false);
//   delay(1000);
//   arm.setStopping(coast);
//   arm.spin(reverse);
//   delay(300);
//   move.straighth(-10, chassis.h, YKP, YKD, TKP, TKD, 7, 12, 12, 0.5);
//   arm.stop();

//   // return to start
//   move.facePoint(0, 0, TKP, TKI, TKD, 6, 6, true);
//   move.bto(0, 0, 7, 12, YKP, TKP, 3, 6);
//   NeutralLock.set(0);

//   // first mogo

//   // face b to 15.9, 5.8 
//   move.facePoint(15.9, 5.8, TKP/**1.2*/, TKI, TKD/**1.2*/,  12, 6, true);
//   delay(100);
//   //move.bto(27.7, 4.9, 4, 12, YKP, TKP, 12, 7);
//   move.straighthToPoint(-15, 15.9, 5.8, YKP, YKD, TKP, TKD, 4, 12, 12, 5.4, true);

//   BackMech.set(1);
//   move.voldrive(0, 0);
//   delay(400); // no swing

//   // clamped first mogo

//   // face to 23.5, 41.9 
//   move.face(270, TKP, TKI, TKD, 12, 8);
//   move.facePoint(23.5, 41.9, TKP, TKI, TKD,  12, 6, false);
//   intake.spin(forward);
//   delay(200); // preload ring 1
//   move.to(23.5, 41.9, 7, 12, YKP, TKP, 12, 6); // ring 2
//   delay(200);

//   // face to 59.8, 40.4
//   move.facePoint(59.8, 40.4, TKP, TKI, TKD, 6, 6, false);
//   delay(200);
//   move.to(59.8, 40.4, 7, 12, YKP, TKP, 3, 6);
//   delay(200);

//   // face to 56.9, -5.8 
//   move.facePoint(56.9, -5.8, TKP*1.6, TKI, TKD, 6, 6, false);
//   delay(200);
//   move.to(56.9, -5.8, 3, 12, YKP*0.8, TKP*1.2, 3, 9);
//   //move.voldrive(4, 4);
//   delay(700);

//   // face b to 75.8, 48.8
//   //move.facePoint(74.3, 24.2, TKP, TKI, TKD, 12, 8, true);
//   move.bto(56.8, 48.8, 7, 12, YKP, TKP*0.8, 7, 6);
//   // x was 75.8

//   // face to 74.5, -4.8 
//   move.facePoint(74.5, -4.8, TKP, TKI, TKD, 12, 6, false);
//   delay(200);
//   move.to(74.5, -4.8, 7, 12, YKP, TKP, 12, 9);
//   move.voldrive(0, 0);
//   delay(1000);

//   // drop mogo off
//   // move.face(330, TKP, TKI, TKD, 12, 5.5);
//   // delay(300);
//   // intake.spin(reverse);
//   // delay(200);
//   // BackMech.set(0);
//   // move.voldrive(-6, -6);
//   // delay(500);
//   // intake.stop();
//   // move.voldrive(6, 6);
//   // delay(200);

//   // b to 51.2, 14.9
//   move.bto(51.2, 14.9, 7, 12, YKP, TKP*0.8, 7, 6);
//   intake.spin(reverse);
//   // b face to 67.8, -3.6
//   move.facePoint(67.8, -3.6, TKP, TKI, TKD, 7, 6, true);
//   delay(200);
//   move.bto(67.8, -3.6, 5, 12, YKP, TKP, 7, 9);
//   delay(200);
//   BackMech.set(0);
//   delay(200);
//   intake.stop();


//   // SECOND MOGO ////////////////////////////////////////////////////////////////////////
//   // face to -18.9, 26.0
//   // y was 22
//   move.facePoint(-23.9, 18.0, TKP, TKI, TKD, 7, 6, false);
//   delay(200);
//   move.to(-23.9, 18.0, 7, 12, YKP, TKP, 7, 6);
//   delay(500);

//   enableStopper = true;
//   holdRing();

//   // face to -37.9, 38.0  
//   move.facePoint(-44.9, 38.0, TKP, TKI, TKD, 7, 6, false);
//   delay(200);
//   move.to(-44.9, 38.0, 7, 12, YKP, TKP, 7, 9);
//   delay(200);

//   // face b to -41.1, 27.5
//   move.facePoint(-48.1, 27.5, TKP*1.4, TKI, TKD, 7, 6, true);
//   delay(200);
//   move.bto(-48.1, 27.5, 7, 12, YKP, TKP*1.2, 7, 9);
//   delay(200);

//   move.face(0, TKP, TKI, TKD, 7, 5.5);
//   delay(300);
//   move.straighth(-20, 0, YKP, YKD, TKP, TKD, 4, 12, 12, 3);
//   //move.bto(-42.9, 20.5, 4, 12, YKP, TKP*0.8, 7, 6);


//   BackMech.set(1); 
//   move.voldrive(0, 0);
//   delay(400); // no swing

//   // face to -69.3, 28.6
//   move.facePoint(-74.3, 32.6, TKP, TKI, TKD, 7, 6, false); // y was 34.6
//   delay(200);
//   enableStopper = false;
//   continueRing();
//   intake.spin(forward);
//   delay(400);
//   move.to(-74.3, 32.6, 9, 12, YKP, TKP, 7, 6);
//   delay(800);
//   // face to -87.4, 62.0
//   move.facePoint(-94.4, 67.0, TKP*1.2, TKI, TKD, 7, 6, false);
//   delay(200);
//   move.to(-94.4, 67.0, 7, 12, YKP, TKP, 7, 6);
//   delay(2000);

// // // NEUTRAL STAKE ///////////////////////////////////////////////////////////////
// //   NeutralLock.set(1);
// //   delay(100);
// //   arm.setStopping(hold);
// //   arm.setVelocity(100, percent);
// //   arm.spinFor(forward, 2700, degrees, true);

// //   // face b to -83.4, 63.6
// //   move.facePoint(-83.4, 67.1, TKP*1.2, TKI, TKD, 7, 6, true);
// //   move.bto(-83.4, 67.1, 7, 12, YKP, TKP*1.2, 7, 6);

// //   move.face(270, TKP*0.9, TKI, TKD, 12, 6);
// //   delay(300);

// //   move.voldrive(4, 4);
// //   delay(600);

// //   arm.spinFor(reverse, 1200, degrees, true);
// //   arm.setStopping(coast);
// //   arm.stop();
// //   delay(300);
// //   move.straighth(-10, chassis.h, YKP, YKD, TKP, TKD, 7, 12, 12, 0.5);
// //   enableNeutralStopper = false;
// //   intake.spin(forward);
// // //////////////////////////////////////////////////////////////////////////////////

//   // b to -72.2, 40.5
//   move.bto(-72.2, 40.5, 7, 12, YKP, TKP*0.8, 7, 6);
//   // face to -74.2, 5.7
//   move.facePoint(-74.2, 5.7, TKP, TKI, TKD, 7, 6, false);
//   delay(200);
//   move.to(-74.2, 5.7, 9, 12, YKP, TKP, 7, 6);
//   // face to -74.0, -11.0
//   move.facePoint(-74.0, -11.0, TKP, TKI, TKD, 7, 6, false);
//   delay(200);
//   move.to(-74.0, -11.0, 9, 12, YKP, TKP, 7, 6);
//   // b to -89.0, 46.1
//   move.bto(-89.0, 46.1, 7, 12, YKP, TKP*0.8, 7, 6);
//   // face to -93.1, -11.6 
//   move.facePoint(-93.1, -11.6, TKP, TKI, TKD, 7, 6, false);
//   delay(200);
//   move.to(-93.1, -11.6, 9, 12, YKP, TKP, 7, 6);
//   delay(800);
  
//   // drop off mogo
//   // move.face(90, TKP, TKI, TKD, 12, 10);
//   // move.face(32, TKP, TKI, TKD, 12, 5.5);
//   // delay(300);
//   // intake.spin(reverse);
//   // delay(200);
//   // BackMech.set(0);
//   // move.voldrive(-6, -6);
//   // delay(650);
//   // intake.stop();
//   // move.voldrive(6, 6);
//   // delay(400);

//   // b to -76.8, 11.7
//   move.bto(-76.8, 11.7, 7, 12, YKP, TKP*0.8, 7, 6);
//   intake.spin(reverse);
//   // b face to -90.3, -8.2
//   move.facePoint(-90.3, -8.2, TKP, TKI, TKD, 7, 6, true);
//   delay(200);
//   move.bto(-90.3, -8.2, 5, 12, YKP, TKP, 7, 9);
//   delay(200);
//   BackMech.set(0);
//   delay(200);


//   // third mogo ///////////////////////////////////////////////////////////////////////////////////////////////
//   // face to -79.6, 73.3
//   move.facePoint(-79.6, 73.3, TKP, TKI, TKD, 7, 6, false);
//   delay(200);
//   move.to(-79.6, 73.3, 7, 12, YKP, TKP, 7, 6);
//   enableStopper = true;
//   holdRing();

//   // face to -47.1, 100.0
//   move.facePoint(-47.1, 100.0, TKP, TKI, TKD, 7, 6, false);
//   //delay(200);
//   move.to(-47.1, 100.0, 7, 12, YKP, TKP, 7, 6);

//   // b face to -29.4, 124.4 
//   move.facePoint(-29.4, 124.4, TKP, TKI, TKD, 7, 6, true);
//   delay(200);
//   move.bto(-29.4, 124.4, 5, 12, YKP, TKP, 7, 9);
//   delay(200);

//   // b face to -14.4, 133.7
//   move.facePoint(-14.4, 133.7, TKP, TKI, TKD, 7, 6, true);
//   delay(200);
//   move.bto(-14.4, 133.7, 5, 12, YKP, TKP, 7, 9);
//   delay(200);

//   BackMech.set(1); 
//   move.voldrive(0, 0);
//   delay(400); // no swing

//   // face to -75.5, 101.5
//   move.facePoint(-80.5, 97.5, TKP, TKI, TKD, 7, 6, false);
//   delay(200);
//   continueRing();
//   enableStopper = false;
//   intake.spin(forward);
//   delay(800);
//   move.to(-80.5, 97.5, 7, 12, YKP, TKP, 7, 6);
//   delay(800);

//   // b face to -13.7, 137.4
//   move.facePoint(-13.7, 137.4, TKP, TKI, TKD, 7, 6, true);
//   move.bto(-13.7, 137.4, 5, 12, YKP, TKP, 7, 9);
//   delay(200);


//   // // under tower
//   // // face to -33.9, 85.1
//   // move.facePoint(-33.9, 85.1, TKP, TKI, TKD, 7, 6, false);
//   // //delay(200);
//   // move.to(-33.9, 85.1, 7, 12, YKP, TKP, 7, 6); 
//   // // face to -18.8, 69.3
//   // move.facePoint(-14.8, 65.3, TKP, TKI, TKD, 7, 6, false);
//   // //delay(200);
//   // move.to(-14.8, 65.3, 7, 12, YKP, TKP, 7, 6); 
//   // // intake back on but with ring stop
//   // enableStopper = false;
//   // intake.spin(forward);
//   // delay(700);
//   // enableStopper = true;
//   // holdRing();
//   // // face to -10.0, 61.3
//   // move.facePoint(-10.0, 61.3, TKP, TKI, TKD, 7, 6, false);
//   // //delay(200);
//   // move.to(-10.0, 61.3, 7, 12, YKP, TKP, 7, 6);
//   // enableStopper = false;
//   // continueRing();
//   // intake.spinFor(400, degrees, true);

//   // // out of gate
//   // // face to 18.2, 94.1
//   // move.facePoint(24.2, 96.1, TKP, TKI, TKD, 7, 6, false);
//   // //delay(200);
//   // move.to(24.2, 96.1, 7, 12, YKP, TKP, 7, 6); 

//   // // re enable intake
//   // intake.spin(forward);
//   // delay(400); // was 800

//   // // face to 25.4, 101.8
//   // move.facePoint(25.4, 101.8, TKP, TKI, TKD, 7, 6, false);
//   // //delay(200);
//   // move.to(25.4, 101.8, 7, 12, YKP, TKP, 7, 6); 

//   // // face to 48.0, 96.0
//   // move.facePoint(48.0, 96.0, TKP, TKI, TKD, 7, 6, false);
//   // //delay(200);
//   // move.to(48.0, 96.0, 7, 12, YKP, TKP, 7, 6); 

//   // // face to 64.4, 62.6
//   // move.facePoint(64.4, 62.6, TKP, TKI, TKD, 7, 6, false);
//   // //delay(200);
//   // move.to(64.4, 62.6, 7, 12, YKP, TKP, 7, 6); 
//   // delay(700);



//   autonStop();
// }

// // void MOASkills(){
// //   delay(5000);
// //   NeutralLock.set(0);
// //   auto_settings();
// //   starttime = Brain.timer(msec);
// //   intake_lift = false;
// //   chassis.setPos(0, 0, 270);


// //   move.cataEnabled = false;

// //   intake.setVelocity(100, percent);

// //   // face b to 26.7, 7.7 
// //   move.facePoint(27.7, 4.9, TKP*1.2, TKI, TKD*1.2,  12, 6, true);
// //   delay(100);
// //   //move.bto(27.7, 4.9, 4, 12, YKP, TKP, 12, 7);
// //   move.straighthToPoint(-15, 27.7, 4.9, YKP, YKD, TKP, TKD, 4, 12, 12, 5.4, true);

// //   BackMech.set(1);
// //   move.voldrive(0, 0);
// //   delay(400); // no swing

// //   // clamped first mogo

// //   // face to 31.9, 43.7 
// //   move.face(270, TKP, TKI, TKD, 12, 8);
// //   move.facePoint(31.9, 43.7, TKP, TKI, TKD,  12, 6, false);
// //   intake.spin(forward);
// //   delay(200); // preload ring 1
// //   move.to(31.9, 43.7, 7, 12, YKP, TKP, 12, 6); // ring 2
// //   delay(200);

// //   // face to 78.9, 35.5
// //   move.facePoint(72.9, 35.5, TKP, TKI, TKD, 6, 6, false);
// //   delay(200);
// //   move.to(72.9, 35.5, 7, 12, YKP, TKP, 3, 6);
// //   delay(200);

// //   // face to 62.5, -6.6 
// //   move.facePoint(60.5, -9.6, TKP*1.6, TKI, TKD, 6, 6, false);
// //   delay(200);
// //   move.to(60.5, -9.6, 3, 12, YKP*0.8, TKP*1.2, 3, 9);
// //   move.voldrive(4, 4);
// //   delay(700);

// //   // face b to 74.3, 24.2
// //   //move.facePoint(74.3, 24.2, TKP, TKI, TKD, 12, 8, true);
// //   move.bto(77.3, 34.2, 7, 12, YKP, TKP*0.8, 7, 6);

// //   // face to 76.1, -18.8 
// //   move.facePoint(76.1, -18.8, TKP, TKI, TKD, 12, 6, false);
// //   delay(200);
// //   move.to(76.1, -18.8, 7, 12, YKP, TKP, 12, 6);
// //   move.voldrive(0, 0);
// //   delay(1000);

// //   // drop mogo off
// //   move.face(330, TKP*1.2, TKI, TKD*1.2, 12, 5.5);
// //   delay(300);
// //   intake.spin(reverse);
// //   delay(200);
// //   BackMech.set(0);
// //   move.voldrive(-6, -6);
// //   delay(500);
// //   intake.stop();


// //   // SECOND MOGO ////////////////////////////////////////////////////////////////////////////
// //   // // to 49.0, 11.7
// //   // move.to(49.0, 11.7, 6, 12, YKP*0.8, TKP*1.5, 12, 3);
// //   // // y speed was 7
// //   // // move.face(90, TKP, TKI, TKD, 12, 5.5);
// //   // // delay(200);
// //   // // move.voldrive(0, 0);
// //   // // delay(100);
// //   // // b to 0.0, 12.7
// //   // move.facePoint(-1.5, 13.1, TKP, TKI, TKD, 12, 6, true);
// //   // delay(300);
// //   // move.bto(-1.5, 13.1, 7, 12, YKP*0.8, TKP*1.35, 12, 6);
// //   // delay(400); // settle
// //   // // was x 6.5 (worked but too much adjustment issue)
// //   // // swing on right 65.4
// //   // move.swingOnRight(69.4, TKP*0.7, TKI, TKD, 12, 6);

// //   // move.straighth(-21, 69.4, YKP, YKD, TKP, TKD, 6, 12, 12, 6);

// //   // // REDO

// //   // // to -13.4, 7.1 
// //   // move.to(-13.4, 7.1, 12, 12, YKP*0.8, TKP*1.5, 12, 3);
// //   // move.face(90, TKP*1.3, TKI, TKD*1.2, 12, 5.5);
// //   // delay(400);
// //   // // b to -18.4, 13.0
// //   // move.bto(-18.4, 8.1, 7, 12, YKP*0.8, TKP*1.5, 12, 7);
// //   // move.face(130, TKP*1.3, TKI, TKD, 3, 5.5);
// //   // move.face(90, TKP*1.3, TKI, TKD*1.22, 12, 5.5);
// //   // move.straighth(-17, 90, YKP, YKD, TKP, TKD, 6, 12, 12, 6);

// //   // face to -16.1, 18.5
// //   move.facePoint(-16.1, 18.5, TKP, TKI, TKD, 12, 6, false);
// //   move.to(-16.1, 18.5, 12, 12, YKP*0.8, TKP*1.5, 12, 6);
// //   delay(1500);

// //   // face b to -8.7, 13.3
// //   move.facePoint(-8.7, 13.3, TKP, TKI, TKD, 12, 6, true);
// //   delay(1500);
// //   move.bto(-8.7, 13.3, 12, 12, YKP*0.8, TKP*1.5, 12, 6);

// //   autonStop();

// //   // face b to

// //   BackMech.set(1);
// //   move.voldrive(0, 0);
// //   delay(400); // no swing

// //   // intake face to -30.6, 44.9  
// //   intake.setVelocity(100, percent);
// //   intake.spin(forward);
// //   move.facePoint(-30.6, 44.9, TKP, TKI, TKD, 12, 6, false);
// //   delay(200);
// //   move.to(-30.6, 44.9, 7, 12, YKP*0.8, TKP*1.5, 12, 6);

// //   // face to -84.1, 71.0 
// //   move.facePoint(-84.1, 76.0, TKP, TKI, TKD, 12, 6, false);
// //   delay(200);
// //   move.to(-84.1, 73.0, 8, 12, YKP*0.8, TKP*1.5, 12, 6);
// //   move.voldrive(-0.5, -0.5);
// //   delay(200);

// //   move.straighth(-10, 270, YKP, YKD, TKP*1.5, TKD, 6, 12, 12, 6);

// //   // face to -65.3, 31.6
// //   move.facePoint(-65.3, 31.6, TKP*1.2, TKI, TKD, 12, 6, false);
// //   delay(200);
// //   move.to(-65.3, 31.6, 4, 12, YKP*0.8, TKP*1.35, 12, 3);

// //   // face to -66.2, -22.3  
// //   move.facePoint(-68.2, -20.3, TKP*1.6, TKI, TKD, 12, 3, false);
// //   delay(200);
// //   move.to(-68.2, -20.3, 4, 12, YKP*0.8, TKP*1.5, 12, 9);
// //   move.voldrive(0, 0);
// //   delay(500);

// //   // b to -78.0, 36.9
// //   move.bto(-78.0, 36.9, 7, 12, YKP, TKP*1.2, 7, 6);

// //   // face to -85.6, -9.4 
// //   move.facePoint(-85.6, -9.4, TKP, TKI, TKD, 12, 6, false);
// //   delay(200);
// //   move.to(-85.6, -9.4, 7, 12, YKP, TKP, 12, 6);
// //   move.voldrive(0, 0);
// //   delay(700);

// //   // drop second mogo off
// //   move.face(90, TKP, TKI, TKD, 12, 10);
// //   move.face(25, TKP, TKI, TKD, 12, 5.5);
// //   delay(300);
// //   intake.spin(reverse);
// //   delay(200);
// //   BackMech.set(0);
// //   move.voldrive(-6, -6);
// //   delay(650);
// //   intake.stop();

// //   // THIRD MOGO ///////////////////////////////////////////////////////////////////
// //   // to -70.3, 57.4
// //   move.to(-70.3, 62.4, 12, 12, YKP*0.8, TKP*1.5, 12, 6);
// //   // to -32.6, 89.6
// //   holdRing();
// //   move.facePoint(-32.1, 106.2, TKP, TKI, TKD, 12, 6, false);
// //   move.to(-32.1, 106.2, 7, 12, YKP*0.8, TKP*1.35, 12, 5);
// //   move.voldrive(0,0);
// //   delay(400);
// //   //move.face(270, TKP, TKI, TKD, 12, 10); // switch (placehold for coord)

// //   // b face to -34.6, 109.2
// //   move.facePoint(-34.6, 109.2, TKP, TKI, TKD, 12, 6, true);
// //   move.bto(-34.6, 109.2, 7, 12, YKP*0.8, TKP*1.35, 12, 9);
// //   // face 236
// //   move.swingOnLeft(238, TKP*0.7, TKI, TKD, 8, 3);

// //   move.straighth(-30, 238, YKP, YKD, TKP*1.2, TKD, 6, 12, 12, 6);

// //   vex::task stop(stopRing);

// //   BackMech.set(1);
// //   move.voldrive(0, 0);
// //   delay(400); // no swing


// //   // face to -71.2, 98.6
// //   move.facePoint(-74.2, 98.6, TKP, TKI, TKD, 12, 6, false);
// //   intake.spin(forward);
// //   delay(500);
// //   move.to(-74.2, 98.6, 8, 12, YKP*0.8, TKP*1.5, 12, 3);

  


// // autonStop();
// // /*
// // //////////////// NEUTRALS /////////////////////
// //   // face to -67.2, 113.7
// //   move.facePoint(-67.2, 113.7, TKP, TKI, TKD,  12, 6, false);
// //   NeutralLoad();
// //   move.to(-67.2, 113.7, 7, 12, YKP, TKP*0.4, 12, 4);
// //   move.voldrive(-0.5, -0.5);
// //   delay(2000); 
// //   // face to -24.8, 102.4
// //   move.facePoint(-24.8, 102.4, TKP, TKI, TKD,  12, 6, false);
// //   NeutralLoad();
// //   move.to(-24.8, 102.4, 4, 12, YKP, TKP*0.4, 12, 4);
// //   move.voldrive(-0.5, -0.5);
// //   delay(2000); 
// //   vex::task stop(RingDetector);
// //   NeutralLock.set(1);
// //   armUp();

// //   // face to -79.9, 72.8
// //   move.facePoint(-79.9, 67.8, TKP, TKI, TKD, 7, 6, false);
// //   move.to(-79.9, 67.8, 4, 12, YKP, TKP, 6, 7);
// //   move.face(270, TKP*1.2, TKI, TKD*1.2, 6, 4.5);
// //   // to 87.7, 65.8  
// //   //move.to(94.7, 65.8, 4, 12, YKP, TKP, 6, 6);
// //   move.straighth(25, 270, YKP, YKD, TKP, TKD, 6, 12, 12, 0);
// //   delay(1000);
// //   vex::task::stop(SlowRaise);
// //   arm.setVelocity(100, percent);
// //   arm.spin(reverse);
// //   delay(500);
// //   arm.stop(coast);
// //   NeutralLock.set(0);
// //   delay(400);
// //   move.straighth(-15, 270, YKP, YKD, TKP, TKD, 6, 12, 12, 6);

// // //////////////////////////////////////////////////////////////////////////////
// // // OLD PATH
// // //////////////////////////////////////////////////////////////////////////////
// //   // neutral load 
// //   // face to 75.8, 84.8
// //   move.facePoint(78.8, 84.8, TKP, TKI, TKD,  12, 6, false);
// //   NeutralLoad();
// //   move.to(78.8, 84.8, 7, 12, YKP, TKP*0.4, 12, 4);
// //   move.voldrive(-0.5, -0.5);
// //   delay(2000);
// //   // face to 53.7, 115.5 
// //   move.facePoint(53.7, 115.5, TKP, TKI, TKD, 5, 6, false);
// //   NeutralLoad();
// //   move.to(53.7, 115.5, 7, 12, YKP, TKP, 12, 4);
// //   move.voldrive(0, 0);
// //   stopdrive(true);
// //   delay(2000);
// //   stopdrive(false);

// //   // neutral stake
// //   NeutralLock.set(1);
// //   // b face to 61.5, 80.7
// //   move.facePoint(61.5, 80.7, TKP*1.2, TKI, TKD*1.2, 8, 6, true);
// //   delay(100);
// //   vex::task::stop(RingDetector);
// //   armUp();
// //   move.bto(61.5, 77.7, 4, 12, YKP, TKP, 6, 7);
// //   move.face(90, TKP*1.2, TKI, TKD*1.2, 6, 4.5);
// //   // to 87.7, 65.8  
// //   //move.to(94.7, 65.8, 4, 12, YKP, TKP, 6, 6);
// //   move.straighth(25, 90, YKP, YKD, TKP, TKD, 6, 12, 12, 0);
// //   delay(1000);
// //   vex::task::stop(SlowRaise);
// //   arm.setVelocity(100, percent);
// //   arm.spin(reverse);
// //   delay(500);
// //   arm.stop(coast);
// //   NeutralLock.set(0);
// //   delay(400);
// //   move.straighth(-15, 90, YKP, YKD, TKP, TKD, 6, 12, 12, 6);
// // */





// //   autonStop();
// // }

// MOA Elim
  // // SECTION 2 (mobile goal) ////////////////////////////////////////////////////////
  // move.facePoint(4.0*ElimFlip, 32.6, TKP, TKI, TKD*1.3, 12, 4, true);
  // move.bto(4.0*ElimFlip, 32.6, 12, 12, YKP, TKP*1.3, 12, 3);
  // move.voldrive(-2, -2);
  // delay(600);
  // BackMech.set(1);
  // move.veldrive(5, 5);
  // delay(30);
  // move.voldrive(0, 0);
  // delay(200);
  // intake.spin(forward);
  // delay(500);
  // move.facePoint(29.2*ElimFlip, 45.8, TKP*1.2, TKI, TKD*1.3, 8, 6, false);
  // //intake.spin(forward);
  // move.to(29.2*ElimFlip, 45.8, 7, 12, YKP, TKP*1.2, 12, 6);
  // //intake.spin(forward);
  // move.face(-260*ElimFlip, TKP, TKI, TKD*1.3, 12, 1);
  // // enableStopper = true;
  // // holdRing();
  // move.straighth(8, -260*ElimFlip, YKP, YKD*.8, TKP*1.3, TKD, 6, 12, 12, 2);
  // move.voldrive(-2, -2);
  // delay(250);
  // delay(800);
  // move.voldrive(0, 0);
  // BackMech.set(0);
  // move.voldrive(2, 2);
  // delay(150);
  // move.voldrive(0, 0);
  // /////////////////////////////////////////////////////////////////////////////////

  // void PPSSkills2(){
//   delay(7000);
//   Stick.set(0);
//   NeutralLock.set(0);
//   auto_settings();
//   starttime = Brain.timer(msec);
//   intake_lift = false;
//   chassis.setPos(0, 0, 0);
//   TKP = 1.1*0.4; //was 1.1 
//   YKP = 2.60; // was 2.60
//   move.cataEnabled = false;

//   enableNeutralStopper = false;
//   enableStopper = false;

//   delay(200);

//   intake.setVelocity(100, percent);
//   move.straighth(-10, chassis.h, YKP, YKD, TKP, TKD, 4, 12, 12, 0);
//   delay(600);
//   intake.spin(forward);
//   delay(700);
//   intake.stop();

//   // pps to -0.5, 9.3 relative ang 90
//   move.PPSto(-0.5, 9.3, 12, 12, YKP, TKP, 12, 3/*BL*/, 90, 3, 14);
//   intake.setVelocity(100, percent);
//   intake.spin(forward);
//   enableStopper = true;
//   holdRing();

//   // pps to 35.1, 43.5 
//   move.PPSto(35.1, 43.5, 7.5, 7.5, YKP, TKP, 12, 8/*BL*/, 8/*RAng*/, 6/*TE*/, 22/*Look*/);

//   // b pps to 35.5, 14.0
//   move.PPSbto(35.5, 8.0, 7.5, 7.5, YKP, TKP*0.6, 4, 3/*BL*/, 6, 8, 22);
//   move.voldrive(-3, -3);
//   delay(400);
//   BackMech.set(1);
//   delay(200);
//   enableStopper = false;
//   continueRing();
//   intake.spin(reverse);

//   // pps to 53.1, 76.1 while ejecting ring
//   move.PPSto(53.1, 76.1, 8, 8, YKP, TKP, 12, 8/*BL*/, 8/*RAng*/, 6/*TE*/, 22/*Look*/);
//   delay(300);
//   enableStopper = true;
//   holdRing();
//   intake.spin(forward);
//   NeutralLock.set(1);

//   // pps to 78.4, 74.2 and hold ring
//   move.PPSto(76.0, 74.2, 7, 7, YKP, TKP, 12, 11/*BL*/, 8/*RAng*/, 6/*TE*/, 22/*Look*/);
//   delay(500);

//   // neutral stake ring 1
//   // b pps to 66.8, 72.4
//   intake.stop();
//   arm.setVelocity(100, percent);
//   arm.spinFor(2700, degrees, false);
//   move.PPSbto(68.8, 72.4, 6, 6, YKP, TKP*1.23, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 16/*Look*/);
//   delay(500);
//   // pps to 81.9, 72.4
//   move.PPSto(81.9, 72.4, 6, 6, YKP, TKP*1.23, 12, 3/*BL*/, 4/*RAng*/, 6/*TE*/, 22/*Look*/);

//   move.face(90, TKP, TKI, TKD, 12, 3);
//   move.voldrive(4, 4);
//   delay(700);
//   arm.spinFor(-1200, degrees, false);
//   delay(500);
//   NeutralLock.set(0);
//   arm.setStopping(coast);
//   arm.stop();

//   // b pps to 62.9, 72.1
//   move.PPSbto(62.9, 72.1, 5.5, 5.5, YKP, TKP*0.6, 12, 3/*BL*/, 6, 8, 22);
//   enableStopper = false;
//   continueRing();
//   intake.spin(forward);

//   // pps to 25.6, 110.7
//   move.PPSto(25.6, 110.7, 7.5, 7.5, YKP, TKP, 12, 8/*BL*/, 12/*RAng*/, 6/*TE*/, 22/*Look*/);

//   arm.setStopping(hold);
//   arm.spinFor(1800, degrees, false);
//   // pps to 77.4, 105.6
//   move.PPSto(80.4, 105.6, 7.5, 7.5, YKP, TKP, 12, 8/*BL*/, 12/*RAng*/, 6/*TE*/, 22/*Look*/);
//   delay(600);
//   intake.setVelocity(40, percent);

//   // pps to 81.0, 146.9
//   move.PPSto(81.0, 143.9, 12, 12, YKP, TKP, 12, 9/*BL*/, 17/*RAng*/, 6/*TE*/, 22/*Look*/);
//   //intake.setVelocity(100, percent);

//   // pps to 60.2, 150.7
//   move.PPSto(60.2, 150.7, 7.5, 7.5, YKP, TKP, 12, 8/*BL*/, 12/*RAng*/, 6/*TE*/, 22/*Look*/);

//   // b pps to 70.5, 152.5
//   move.PPSbto(70.5, 152.5, 6, 6, YKP, TKP*1.1, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 16/*Look*/);

//   // pps to 65.5, 136.8
//   move.PPSto(65.5, 136.8, 7.5, 7.5, YKP, TKP, 12, 8/*BL*/, 12/*RAng*/, 6/*TE*/, 22/*Look*/);
//   delay(400);
//   intake.setVelocity(100, percent);
//   delay(1200);
//   BackMech.set(0);
//   delay(200);
//   move.face(225, TKP, TKI, TKD, 12, 3);
//   move.voldrive(-4, -4);
//   delay(1500);
//   arm.setStopping(coast);
//   arm.stop();

//   // finished with first neutral stake and first goal at this point

//   // pps to 39.1, 122.8
//   move.PPSto(39.1, 122.8, 7.5, 7.5, YKP, TKP, 12, 8/*BL*/, 12/*RAng*/, 6/*TE*/, 22/*Look*/);
//   delay(500);

//   // second goal

//   // b pps to 5.2, 134.8 
//   move.PPSbto(5.2, 134.8, 6, 6, YKP, TKP*0.8, 4, 6/*BL*/, 6, 8, 22);
//   move.voldrive(-2, -2);
//   delay(400);
//   BackMech.set(1);
//   delay(200);
//   enableStopper = true;
//   holdRing();
//   intake.spin(forward);

//   // pps to -35.5, 99.9
//   move.PPSto(-35.5, 99.9, 7.5, 7.5, YKP, TKP, 12, 8/*BL*/, 12/*RAng*/, 6/*TE*/, 22/*Look*/);
//   delay(1000);
//   enableStopper = false;
//   continueRing();
//   intake.spin(reverse);
//   delay(600);
//   intake.spin(forward);

//   // pps to -75.1, 100.6
//   move.PPSto(-75.1, 100.6, 7.5, 7.5, YKP, TKP, 12, 8/*BL*/, 12/*RAng*/, 6/*TE*/, 22/*Look*/);
//   delay(700);
//   NeutralLock.set(1);

//   // b pps to -54.7, 72.2
//   move.PPSbto(-54.7, 68.2, 6, 6, YKP, TKP*1.23, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 16/*Look*/);

//   // pps to -77.9, 67.3
//   move.PPSto(-75.9, 68.3, 7, 7, YKP, TKP, 12, 11/*BL*/, 8/*RAng*/, 6/*TE*/, 22/*Look*/);
//   delay(500);

//   // b pps to -68.0, 70.1 neutral stake 2
//   intake.stop();
//   arm.setVelocity(100, percent);
//   arm.spinFor(2700, degrees, false);
//   move.PPSbto(-68.0, 68.1, 6, 6, YKP, TKP*1.1, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 16/*Look*/);
//   delay(500);

//   // pps to -84.2, 71.2
//   move.PPSto(-84.2, 68.3, 6, 6, YKP, TKP*1.23, 12, 3/*BL*/, 7/*RAng*/, 6/*TE*/, 22/*Look*/);

//   move.face(270, TKP, TKI, TKD, 12, 3);
//   move.voldrive(4, 4);
//   delay(700);
//   arm.spinFor(-900, degrees, false);
//   delay(400);
//   NeutralLock.set(0);
//   arm.setStopping(coast);
//   arm.stop();

//   move.PPSbto(-68.0, 67.1, 6, 6, YKP, TKP*1.1, 12, 3/*BL*/, 10/*RAng*/, 6/*TE*/, 16/*Look*/);
//   intake.spin(forward);

//   // pps to -67.6, 36.6
//   move.PPSto(-67.6, 36.6, 7, 7, YKP, TKP, 12, 11/*BL*/, 8/*RAng*/, 6/*TE*/, 22/*Look*/);

//   // pps to -67.8, -13.3
//   move.PPSto(-67.8, -13.3, 7, 7, YKP, TKP, 12, 8/*BL*/, 8/*RAng*/, 6/*TE*/, 22/*Look*/);

//   // pps to -84.0, 7.9
//   move.PPSto(-84.0, 7.9, 7, 7, YKP, TKP, 12, 10/*BL*/, 8/*RAng*/, 6/*TE*/, 22/*Look*/);

//   // b pps to -84.1, -9.3(no bl, timeout)
//   move.PPSbto(-84.1, -9.3, 7, 7, YKP, TKP, 12, 0/*BL*/, 8/*RAng*/, 6/*TE*/, 22/*Look*/); 
//   delay(800);
//   BackMech.set(0);


// autonStop();
// }