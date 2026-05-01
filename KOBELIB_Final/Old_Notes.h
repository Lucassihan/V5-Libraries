//   // Programming skills
// void prog() { 
//     move.cataTarg =  66.125;
//   autonStart();
//   auto_settings();
//   starttime = Brain.timer(msec);
//   //xmove.facePoint(0, -24, TKP*1.3, TKI, TKD, 12, 0, true);
//   //delay(1000000);
//   chassis.setPos(0, 0, -8.9);
//   move.cataEnabled = true;
//   int boost_delay = 40;
//   brain_screen_print = false;
 
//   intakeAt(100);
//   piston_boost = true;

//   Brain.Screen.setFillColor(green);
//   Brain.Screen.drawRectangle(0, 0, 480, 240);
//   delay(100);
//   piston_boost = false;
//   delay(900);
//   Brain.Screen.setFillColor(red);
//   Brain.Screen.drawRectangle(0, 0, 480, 240);
//   piston_boost = true;
//   delay(10);
//   move.shoot = true;
//   delay(300);

//   Brain.Screen.setFillColor(green);
//   Brain.Screen.drawRectangle(0, 0, 480, 240);
//   delay(1900);
//   Brain.Screen.setFillColor(red);
//   Brain.Screen.drawRectangle(0, 0, 480, 240);
//   boostShot(boost_delay+30);

//   Brain.Screen.setFillColor(green);
//   Brain.Screen.drawRectangle(0, 0, 480, 240);
//   delay(1900);
//   Brain.Screen.setFillColor(red);
//   Brain.Screen.drawRectangle(0, 0, 480, 240);
//   move.cataTarg =  66.4;
//   boostShot(boost_delay);
  

//   brain_screen_print = true;
//   //starttime = Brain.timer(msec) -( 100 + 900 + 10 + 300 + 1800 + 1800);
//   //starttime = Brain.timer(msec) - (1250 + 320 + 2600 + 410 + 2600 + 410);
//   //autonStop();
// //autonStop();
//   move.bto(3.2, -9, 10, 12, YKP*.8, TKP * 1.3, 2, 2); 
//   move.face(116, TKP, TKI + 0.01, TKD, 12, 10); 
//   intakeAt(100);
//   move.straighth(6, 116, YKP*.8, YKD*.8, TKP, TKD, 9, 12, 12, 4.75);
//   move.straighth(22, 105, YKP*7, YKD*.75, TKP, TKD, 6.5, 10, 12, 2);
//   move.veldrive(-7, -7);
//   delay(30);
//   move.veldrive(0, 0);
//   delay(200);
//   move.bto(21, -15, 9, 12, YKP * 1.1, TKP * 1.3, 12, 2.5);
//   intake_lift = true;
//   move.facePoint(11.75, -55, TKP*1.2, TKI, TKD, 12, 1.5, true);
//   weak_boost(140);

//   //autonStop();
//   move.facePoint(23, 2.7, TKP*1.3, TKI*1.3, TKD*1.5, 9, 11, false);
//   move.to(23.75, 1, 8, 12, YKP * 0.75, TKP*1.3, 12, 2.2); // to triple stack
//   delay(100);
//   intake_lift = false;
//   move.straighth(1.5, chassis.h, YKP*1.1, YKD, TKP, TKD, 2.5, 12, 1, 1);
//   delay(500);
//   intakeAt(90);
//   move.facePoint(10, -57, TKP, TKI + 0.01, TKD, 9, 2, true);
//   move.straighthToPoint(-5, 10, -57, YKP*1.1, YKD, TKP*1.3, TKD, 12, 12, 12, 2.5, true);
//   boostShot(boost_delay); // triple stack boost shot


//   move.facePoint(50.5, -12.7, TKP, TKI + 0.01, TKD - .5, 10, 22, false);
//   intakeAt(95);
//   move.to(49, -13.5, 12, 12, YKP * 1, TKP*1.3, 12, 25); 
//   move.to(76.25, -49, 12, 12, YKP * 1.15, TKP*1.3, 12, 3); 
//   move.facePoint(52, -60.4, TKP*1.6, TKI + 0.01, TKD, 12, 12, true);
//   move.bto(52, -60.5, 12, 12, YKP * 1, TKP*1.3, 12, 3); 
//   move.facePoint(9, -60, TKP, TKI + 0.01, TKD, 9, 2.5, true);
//   move.shoot = true; // 3 in a row on close goal


//   delay(350);
//   move.face(-27, TKP*1.05, TKI + 0.01, TKD, 10, 3); 
//   intakeAt(100);
//   move.straighth(12, -27, YKP, YKD, TKP, TKD, 7, 10, 12, 5);
//   //move.to(45.5, -24, 5.5, 12, YKP*1.1, TKP * 1.3, 12, 2.5); 
//   move.straighth(26, -25, YKP, YKD, TKP, TKD, 6, 12, 12, 2);
//   delay(150);
//   move.bto(50.5, -44, 11, 12, YKP * 1.1, TKP * 1.3, 12, 2.5);
//   move.facePoint(9, -59.5, TKP*1.1, TKI, TKD, 9, 2.75, true);
//   intake_lift = true;
//   move.shoot = true; // close low goal discs 2


//   delay(350);
//   intakeAt(100);
//   drivePID.setConstants(YKP, YKI, YKD + 5);
//   intakeAt(90);
//   move.to(89.5, -44, 11, 12, YKP *1, TKP*1.3, 12, 2.5); 
//   drivePID.setConstants(YKP, YKI, YKD);
//   intake_lift = false;
//   delay(50);
//   move.to(118.25, -42.25, 8, 12, YKP * 0.9, TKP*1.3, .2, 3); 
//   intakeAt(0);
//   move.voldrive(2.5,2.5);
//   delay(300);
//   move.voldrive(.3,5);
//   intakeForBlocking(1150); // first roller


//   move.bto(97, -43, 12, 12, YKP * 1, TKP*1.3, 12, 4); 
//   move.facePoint(90, -64.5, TKP, TKI + 0.01, TKD, 12, 15, false);
//   move.to(90, -64.5, 12, 12, YKP * 0.9, TKP*1.3,  12, 3); 
//   move.voldrive(4, 2.5);
//   delay(200);
//   move.voldrive(5,.3);
//   intakeForBlocking(1150); // second roller

  

//   move.bto(91, -40, 10, 12, YKP * 1, TKP*1.5, 12, 16); 
//   move.bto(115, -3, 9.5, 12, YKP * .8, TKP*1.5, 12, 2.5); 
//   move.face(200, TKP, TKI, TKD, 12, 1.5);
//   move.straighth(-10, 200, YKP, YKD, TKP*1.3, TKD, 7, 12, 1, 6);
//   delay(400);
//   move.face(169.25, TKP*1.2, TKI, TKD, 10, 0);
//   delay(650);
//   move.straighth(5.6, chassis.h-1, YKP*1.3, YKD, TKP*1.3, TKD, 8, 12, 2, .5);
//   move.facePoint(112.25, 46.5, TKP, TKI + 0.01, TKD, 9, 3, true);
//   move.cataTarg =  66.125;
//   boostShot(boost_delay); // far loader shot 1
//   Brain.Screen.setFillColor(green);
//   Brain.Screen.drawRectangle(0, 0, 480, 240);
//   delay(1900);
//   Brain.Screen.setFillColor(red);
//   Brain.Screen.drawRectangle(0, 0, 480, 240);
//   boostShot(boost_delay);

//   Brain.Screen.setFillColor(green);
//   Brain.Screen.drawRectangle(0, 0, 480, 240);
//   delay(1900);
//   Brain.Screen.setFillColor(red);
//   Brain.Screen.drawRectangle(0, 0, 480, 240);
//   move.cataTarg =  66.4;
//   boostShot(boost_delay);


//   move.bto(113, 1, 10, 12, YKP*.9, TKP * 1.3, 3, 2.5); 
//   move.face(304, TKP, TKI + 0.01, TKD, 12, 11); 
//   intakeAt(100);
//   move.straighth(10, 305, YKP*.9, YKD*.8, TKP, TKD, 9, 12, 12, 4.5);
//   move.straighth(21, 294.5, YKP*.825, YKD*.6, TKP, TKD, 6.25, 12, 12, 3);
//   delay(100);
//   //move.to(87.5, 14, 6.75, 12, YKP*.9, TKP * 1.3, 12, 2); 
//   delay(250);
//   move.bto(99.5, 10.5, 9, 12, YKP * 1.1, TKP * 1.3, 12, 2.5);
//   move.facePoint(113.25, 47.5, TKP, TKI + 0.01, TKD, 9, 2.75, true);
//   intake_lift = true;
//   move.shoot = true; // swing turn off lowgoal shot

//   delay(800); 
//   intakeAt(90);
//   move.to(96.2, -8, 8, 12, YKP * 0.8, TKP*1.3, 12, 2); // to triple stack
//   intake_lift = false;
//   delay(50);
//   move.straighth(2.5, chassis.h, YKP, YKD, TKP, TKD, 2.5, 12, 1, 1);
//   delay(550);
//   move.facePoint(112.75, 48, TKP, TKI + 0.01, TKD, 9, 2.5, true);
//   //move.straighthToPoint(-3, 114, 47.5, YKP*1.1, YKD, TKP*1.3, TKD, 12, 12, 12, 2.5, true);
//   boostShot(boost_delay); // triple stack boost shot


//   move.facePoint(72.4, 13.2, TKP, TKI + 0.01, TKD - .5, 10, 25, false);
//   intakeAt(90);
//   move.to(70.4, 13.2, 12, 12, YKP * 1, TKP*1.3, 12, 23); 
//   move.to(42.2, 45.6, 11, 12, YKP * 1, TKP*1.3, 12, 3); 
//   move.facePoint(68.5, 54, TKP*1.4, TKI + 0.01, TKD, 12, 8, true);
//   move.bto(68.5, 54, 12, 12, YKP * 1, TKP*1.3, 12, 2); 
//   move.facePoint(115, 50, TKP, TKI + 0.01, TKD, 9, 3, true);
//   move.shoot = true; // 3 in a row on far goal


//   delay(300);
//   move.face(150, TKP*1.05, TKI + 0.01, TKD, 9, 13); 
//   intakeAt(100);
//   move.straighth(5, 149, YKP, YKD, TKP, TKD, 9, 12, 12, 3);
//   move.straighth(30, 164, YKP, YKD, TKP, TKD, 7, 12, 12, 3);
//   //move.to(73.4, 24.5, 8, 12, YKP, TKP * 1, 12, 2);
//   delay(200);
//   move.bto(69.2, 31.4, 8, 12, YKP * 1.1, TKP * 1.3, 12, 2);
//   move.facePoint(116.74, 52.5, TKP*1.1, TKI, TKD, 9, 2.5, true);
//   move.shoot = true; // far low goal discs 2


//   intake_lift = true;
//   delay(350);
//   intakeAt(90);
//   move.to(31.5, 35.5, 12, 12, YKP * 0.8, TKP*1.3, 12, 2); 
//   intake_lift = false;
//   delay(100);
//   move.to(3, 34.5, 8, 12, YKP * 0.9, TKP*1.3, .2, 3); 
//   intakeAt(0);
//   move.voldrive(3,4);
//   delay(300);
//   move.voldrive(.3,5);
//   intakeForBlocking(1300); // third roller


//   move.bto(24.6, 35, 12, 12, YKP * 0.9, TKP*1.3, 12, 3); 
//   move.facePoint(31, 56.6, TKP*1.1, TKI + 0.01, TKD, 12, 10, false);
//   move.to(31, 56.6, 12, 12, YKP * 0.9, TKP*1.3, 12, 4); 
//   move.voldrive(5,4);
//   delay(200);
//   move.voldrive(5,.3);
//   intakeForBlocking(1300); // fourth roller


//   move.bto(21.4, -2.5, 12, 12, YKP * 1, TKP*1.3, 12, 2.5); 
//   move.facePoint(10, -56, TKP*1.2, TKI, TKD, 12, 1.75, true);
//   boostShot(boost_delay);


//   intakeAt(100);
//   move.to(10, 52.1, 12, 12, YKP, TKP*1.3, 12, 1.5);
//   move.face(314.6, TKP, TKI, TKD, 12, 1);
 

//    side_expand = true;
//   left_expand = true;
//   right_expand = true; // expand

//   delay(250);
//   autonStop();
// }


// void AWP9() {  //new path 9 discs
//   auto_settings();
//    // move.facePoint(0, -24, TKP*1.3, TKI, TKD, 12, 0, true);
//   //delay(1000000);
//   TKD = 5.8;

//   intake_lift = true;
//   chassis.setPos(0, 0, -17.81);
//   move.cataEnabled = true;
//   TKP = TKP*1.15;

//   intakeAt(90);
//   move.straighth(13.2, -17.81, YKP, YKD, TKP*1.3, TKD, 12, 12, 12, 2);
//   delay(50);
//   intake_lift = false;
//   delay(500);
//   move.straighth(-5, -17.81, YKP, YKD*1.2, TKP*1.3, TKD, 12, 12, 12, 1);
//   move.facePoint(-27, 108, TKP, TKI, TKD*1.1, 12, 1, true); // aim at goal triple from the line
//   intakeAt(-100);
//   boostShot(80); // disc rush shot


//   intakeAt(0);
//   move.face(184, TKP, TKI, TKD*1.3, 12, 5);
//   move.straighth(12, 184, YKP, YKD*.8, TKP*1.3, TKD, 6, 12, 12, 0);
//   delay(650);
//   move.voldrive(2, 2);
//   intakeForBlocking(-700); // close roller


//   move.straighth(-5, 180, YKP, YKD*1.2, TKP*1.3, TKD, 5, 12, 12, 2);
//   move.facePoint(13, 13, TKP, TKI, TKD, 12, 4, false);
//   intake_lift = true;
//   intakeAt(90);
//   move.to(13, 13, 11, 12, YKP, TKP*1.3, 12, 3);
//   move.veldrive(-5, -5);
//   delay(30);
//   move.veldrive(0, 0);
//   delay(200);
//   intake_lift = false;
//   move.to(23, 22.5, 6, 12, YKP, TKP * 1.3, .8, 2);
//   delay(200);
//   move.facePoint(-26, 108, TKP, TKI, TKD, 10, .5, true); 
//   intakeAt(-100);
//   boostShot(45); // triple stack shot

//   move.facePoint(67.5, 67.4, TKP*.8, TKI, TKD, 12, 10, false);
//   intakeAt(90);
//   //YKD = YKD *.6;
//   move.to(67.5, 67.4, 10, 12, YKP*.8, TKP * 1.3, 12, 2);
//   //YKD = YKD *1.5;
//   delay(300);
//   move.facePoint(-27, 108, TKP, TKI, TKD, 12, 1.5, true);
//   move.straighthToPoint(-3, -27, 108, YKP, YKD, TKP * 1.3, TKD, 12, 12, 12, 1, true);
//   boostShot(45); // three in a row shot


//   move.facePoint(88.2, 94, TKP, TKI, TKD, 12, 10, false);
//   intakeAt(0);
//   move.to(88.2, 92.5, 12, 12, YKP*.9, TKP * 1.3, 12, 4.5);
//   move.voldrive(2, 3);
//   delay(300);
//   intakeForBlocking(-800); // far roller
//   move.voldrive(0, 0);

// autonStop();
// }

// void closeAuto() {
//  //new path 9 discs
//   auto_settings();
//    // move.facePoint(0, -24, TKP*1.3, TKI, TKD, 12, 0, true);
//   //delay(1000000);
//   TKD = 5.8;

//   intake_lift = true;
//   chassis.setPos(0, 0, -17.81);
//   move.cataEnabled = true;
//   TKP = TKP*1.15;

//   intakeAt(90);
//   move.straighth(13, -17.81, YKP, YKD, TKP*1.3, TKD, 12, 12, 12, 2);
//   delay(50);
//   intake_lift = false;
//   delay(500);
//   move.straighth(-5, -17.81, YKP, YKD*1.2, TKP*1.3, TKD, 12, 12, 12, 1);
//   move.facePoint(-27, 108, TKP, TKI, TKD, 12, 1, true); // aim at goal triple from the line
//   intakeAt(-100);
//   boostShot(80); // disc rush shot


//   intakeAt(0);
//   move.face(184, TKP, TKI, TKD*1.3, 12, 5);
//   move.straighth(12, 184, YKP, YKD*.8, TKP*1.3, TKD, 6, 12, 12, 0);
//   delay(650);
//   move.voldrive(2, 2);
//   intakeForBlocking(-700); // close roller


//   move.straighth(-5, 180, YKP, YKD*1.2, TKP*1.3, TKD, 5, 12, 12, 2);
//   move.facePoint(13.5, 12.7, TKP, TKI, TKD, 12, 4, false);
//   intake_lift = true;
//   intakeAt(90);
//   move.to(13, 13, 11, 12, YKP, TKP, 12, 3);
//   move.veldrive(-7, -7);
//   delay(30);
//   move.veldrive(0, 0);
//   delay(200);
//   intake_lift = false;
//   move.to(23, 22.5, 6, 12, YKP, TKP * 1.3, .8, 2);
//   delay(200);
//   move.facePoint(-26.5, 108, TKP, TKI, TKD, 10, .5, true); 
//   intakeAt(-100);
//   boostShot(45); // triple stack shot
// autonStop();
// }



// void farAuto() {
//   auto_settings();
//   chassis.setPos(0, 0, -43.636);
//   TKD = 5.8;
//   intake_lift = true;
//   move.cataEnabled = true;
//   TKP = TKP*1.15;

//   intakeAt(100);
//   move.straighth(21.2, -43.636, YKP, YKD, TKP*1.3, TKD, 12, 12, 12, 2.5);
//   delay(200);
//   intake_lift = false;
//   delay(600);
//   move.straighth(-5, -43.636, YKP, YKD*1.2, TKP*1.3, TKD, 12, 12, 12, 1);
//   move.facePoint(-115, 35, TKP, TKI, TKD*1.1, 12, 1.5, true); // aim at goal triple from the line
//   intakeAt(-100);
//   boostShot(80); // disc rush shot

//   move.facePoint(.75, 20, TKP, TKI, TKD*1.1, TKD, 7, false); // aim at goal triple from the line
//   //intakeAt(95);
//   intakeAt(0);
//   move.to(.75, 20, 9, 12, YKP, TKP * 1.3, 12, 3);
//   /*
//   delay(400);
//   intakeAt(0);
//   */
//   move.voldrive(4.5, 4.5);
//   delay(500);
//   move.voldrive(3, 1);
//   intakeForBlocking(-800); // roller

//   move.bto(-4.1, 8.4, 10, 12, YKP*1.05, TKP * 1.3, 12, 3);
//   /*
//   move.facePoint(-115, 32, TKP, TKI, TKD, 12, 2, true); // aim at goal triple from the line
//   boostShot(80); // double shot
// */
//   intakeAt(100);
//   move.facePoint(-25.3, -1, TKP, TKI, TKD*1.2, 12, 8, false); // aim at goal triple from the line
//   move.to(-25.3, -1, 10.5, 12, YKP*1, TKP * 1.1, 12, 14);
//   move.to(-45.6, -23.7, 10.5, 12, YKP*1, TKP * 1.5, 12, 2.5);
//   delay(200);
//   move.facePoint(-115, 36.5, TKP, TKI, TKD*1.05, 12, 2, true); // aim at goal triple from the line
//   boostShot(115); // 3 in a line shot

//   move.facePoint(-33.7, -32.4, TKP, TKI, TKD*1.1, 12, 6, false); // aim at goal triple from the line
//   move.to(-33.7, -32.4, 5, 12, YKP*1.05, TKP * 1.3, 12, 3);
//   move.straighth(28, 109, YKP*1, YKD*.6, TKP, TKD, 5.75, 12, 12, 3);
//   delay(400);
//   move.bto(-31, -22.5, 10, 12, YKP*1.05, TKP * 1.3, 1, 2.5);
//   move.facePoint(-115, 36, TKP, TKI, TKD*1.1, 12, 1.5, true); // aim at goal triple from the line
//   boostShot(100); // low goal shot

//   delay(250);
//     autonStop();
// }


// float faceTarg(float xtarg, float ytarg) {
//   float ang = (atan2(xtarg - chassis.x, ytarg - chassis.y) * 180 / M_PI) + 180;

//   return ang;
// }


// void loader_align() {
//   auto_settings();
//   chassis.setPos(0, 0, 0);
//   move.cataEnabled = false;
//   move.face(22, TKP, TKI, TKD, 12, 4);
//   move.straighth(-19, 22, YKP, YKD, TKP*3, TKD, 6, 12, 1, 3);
//   delay(200);
//   move.face(-9, TKP, TKI, TKD, 8, 0);
//   delay(1000);
//   move.straighth(6, -9, YKP, YKD, TKP*1.3, TKD, 7, 12, 1, .5);
//   move.facePoint(2, -65, TKP, TKI, TKD, 12, .5, true);
  
// autonStop();
// }