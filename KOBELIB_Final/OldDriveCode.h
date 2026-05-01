// if(Controller.ButtonLeft.pressing()){
//   trackingWheelLift.set(1);
// }



// // if(endgameDeploy == true){
// //   //poleGrab.set(1);
// //   BarHang.set(1);
// // }

// // if(endgamePull == true){
// //   BarHang.set(0);
// // }



// //    if (Controller.ButtonR1.pressing()) {
// //     /*if (boosted_shots) {
// //     //boostShot(80);
// //     boostShot(45);
// //     //weak_boost(140);
// //     } else {
// //     */

// //     /* actual shots
// //     move.cataEnabled = true;
// //     move.cataTarg = 135.5;
// //     MidLower = false;
// //       move.shoot = true;
// //     */

// //     cata.setVelocity(100, percent);
// //   cata.spin(forward);
// //     }
// //     else {
// //       /*
// // move.cataEnabled = false;
// // move.shoot = false;
// // cata.setVelocity(100, percent);
// // cata.spin(forward);
// // */
// // cata.stop();
// //   }

//       //}  
//   //     else{
//   //   move.cataEnabled = false;
//   // }


// // SKILLS 
// /*
//   if(ConveyorF == true){
//       leftBackWing.set(1);
//       rightBackWing.set(1);
//     }
//     else{
//       leftBackWing.set(0);
//       rightBackWing.set(0);
//     }
// */



// // MATCHES ///////////////////////// 


// //     if(Controller.ButtonA.pressing()){
// //   // rightBackWing.set(1);
// //   // //leftBackWing.set(1);
// // //   chassis.setPos(0, 0, IMU1.heading());
// // //   double TKD = 6.548;
// // // double  TKP = 0.4 * 1.15;
// // // double  TKI = 0.05;
// // //   move.face(0, TKP, TKI, TKD, 12, 0);
// //   // cata.setVelocity(100, percent);
// //   // cata.spin(forward);
// //   fireCata = true;
// //   rightBackWing.set(1); 
// //   //waitUntil(Controller.ButtonY.pressing());
// // }
// //  if (Controller.ButtonY.pressing()){
// //    rightBackWing.set(0);
// //    fireCata = false;
// //   //move.autondriveenabled = false;
// //   cata.stop();

// // }


// //  if (Controller.ButtonR2.pressing()){
// //    leftWing.set(1);
// //    rightWing.set(1);
// //       intake.setVelocity(100, percent);
// //       intake.spin(reverse);
// //       cata.setVelocity(100, percent);
// //       cata.spin(reverse);
// // }
// // else {
// //   leftWing.set(0);
// //   rightWing.set(0);
// //   if(Controller.ButtonL2.pressing() == false){
// //   if(Controller.ButtonL1.pressing() == false){
// //   intake.stop();
// //   cata.stop();
// // }
// // }
// // }

//   if (Controller.ButtonX.pressing() && last_X != 1) {
//     // if (StickSwitch == false) Controller.rumble("-");
//     // if (StickSwitch == true) Controller.rumble(".");

//     StickSwitch = !StickSwitch;
//   }

//   if (Controller.ButtonX.pressing()) {
//     if (StickSwitch == true) {
//     Stick.set(1);
//   }
//   }

//  if (Controller.ButtonX.pressing()) {
//    if (StickSwitch == false) {
//    Stick.set(0);
//   }
//   }

//   last_X = Controller.ButtonX.pressing();

//   if (Controller.ButtonR1.pressing() && last_R2 != 1) {
//     if (HangSwitch == false) Controller.rumble("-");
//     if (HangSwitch == true) Controller.rumble(".");

//     HangSwitch = !HangSwitch;
//   }

//   if (Controller.ButtonR1.pressing()) {
//     if (HangSwitch == true) {
//     BackMech.set(1);
//   }
//   }

//  if (Controller.ButtonR1.pressing()) {
//    if (HangSwitch == false) {
//    BackMech.set(0);
//   }
//   }

//   last_R2 = Controller.ButtonR1.pressing();

// ////////////////////////////////////////////////////////
// // if (Controller.ButtonX.pressing() && last_X != 1) {
// //     if (NeutralLocker == false) Controller.rumble("-");
// //     if (NeutralLocker == true) Controller.rumble(".");

// //     NeutralLocker = !NeutralLocker;
// //   }

// //   if (Controller.ButtonX.pressing()) {
// //     if (NeutralLocker == true) {
// //     NeutralLock.set(1);
// //   }
// //   }

// //  if (Controller.ButtonX.pressing()) {
// //    if (NeutralLocker == false) {
// //    NeutralLock.set(0);
// //   }
// //   }

// //   last_X = Controller.ButtonX.pressing();

//   /////////////////////////////////////////////

// intake.setVelocity(100, percent);
// move.ringDispose();
//   if (Controller.ButtonA.pressing() && last_A != 1) {
//     if (NeutralSwitch == false) Controller.rumble("-");
//     if (NeutralSwitch == true) Controller.rumble(".");

//     NeutralSwitch = !NeutralSwitch;
//   }

//   if (Controller.ButtonA.pressing()) {
//     if (NeutralSwitch == true) {
//     move.neutralMode = true;
//       }
//   }


//   if(forceStop == true){
//     intake.setVelocity(100, percent);
//     intake.spin(reverse);
//   }


//   if(forceDisable == true && forceStop == false){
//     intake.setVelocity(0, percent);
//   }



//  if (Controller.ButtonA.pressing()) {
//    if (NeutralSwitch == false) {
//     move.neutralMode = false;
//   }
//   }

//   last_A = Controller.ButtonA.pressing();

//   if(RingDistance.objectDistance(mm) < 18 && move.neutralMode == true /*&& forceStart == false && forceStop == false*/){
//     ejectRingCommand();
//   }

//       if(ConveyorF == true && forceStop == false && forceStart == false && forceDisable == false && NeutralSwitch == false/*&& ringTracker.value(percent) > 50*/){
//       //matches
//       // 21 to 34
//       intake.setVelocity(100, percent);
//       intake.spin(forward);
//       lockIn();
//     }
//     if(ConveyorR == true && NeutralSwitch == false && forceStop == false && forceStart == false && forceDisable == false){
//       //MATCHES
//       intake.setVelocity(100, percent);
//       intake.spin(reverse);
//     }
//     if (ConveyorF == false && ConveyorR == false && forceStop == false && forceStart == false && forceDisable == false){
//       intake.setStopping(coast);
//       intake.stop();
//     }
//     if(ConveyorF == true && forceStop == false && forceStart == false && forceDisable == false && NeutralSwitch == true/*&& ringTracker.value(percent) > 50*/){
//       //matches
//       // 21 to 34
//       intake.setVelocity(80, percent);
//       intake.spin(forward);
//       NeutralLock.set(0);
//     }

//   if(Controller.ButtonR2.pressing() && Controller.ButtonL1.pressing()){
//     arm.setVelocity(100, percent);
//     arm.spin(forward);
//     arm.setStopping(hold);
//     intake.stop();
//     lockOut();
//   }
//   else if(Controller.ButtonR2.pressing() && Controller.ButtonL2.pressing()){
//     arm.setVelocity(100, percent);
//     arm.setStopping(coast);
//     arm.spin(reverse);
//     intake.stop();
//   }
//   else{
//     arm.stop();
//   }
// //////////////////////////////////////////////







// /*
//     if(driving == true){
//     fr.setStopping(hold);
//     mr.setStopping(hold);
//     fl.setStopping(hold);
//     br.setStopping(hold);
//     ml.setStopping(hold);
//     bl.setStopping(hold);
// */

// /*
//     if(abs(LeftStick)<deadband){
//       fl.setVelocity(0, percent);
//       ml.setVelocity(0, percent);
//       bl.setVelocity(0, percent);
//     }
//     else{
//       fl.setVelocity(LeftStick, percent);
//       ml.setVelocity(LeftStick, percent);
//       bl.setVelocity(LeftStick, percent);
//     }

//     if(abs(RightStick)<deadband){
//       fr.setVelocity(0, percent);
//       mr.setVelocity(0, percent);
//       br.setVelocity(0, percent);
//     }
//     else{
//       fr.setVelocity(RightStick, percent);
//       mr.setVelocity(RightStick, percent);
//       br.setVelocity(RightStick, percent);
//     }

//     fl.spin(reverse);
//     mr.spin(reverse);
//     ml.spin(reverse);
//     bl.spin(reverse);
//     fr.spin(reverse);
//     br.spin(reverse);
//   */
//   // double turnImportance = 0.3;
//   // // was 0.5
  
//   // fr.setStopping(brake);
//   // fl.setStopping(brake);
//   // mr.setStopping(brake);
//   // ml.setStopping(brake);
//   // br.setStopping(brake);
//   // bl.setStopping(brake);
    
//   //   ///////////////////////////////////////////
//   //   //Driver Control
//   //   ////////////////////////////////////////////////////////////////////                                                                                                                                                                                                            /
    
//   //   double turnVal = Controller.Axis1.position(percent);
//   //   double forwardVal = Controller.Axis3.position(percent);

//   //   double turnVolts = turnVal * 0.12;
//   //   double forwardVolts = forwardVal * 0.12 * (1 - (std::abs(turnVolts)/12.0) * turnImportance);



//   //   double leftVolts = (forwardVolts + turnVolts);
//   //   double rightVolts = (forwardVolts - turnVolts);

//   //   fl.spin(forward, leftVolts, voltageUnits::volt);
//   //   ml.spin(forward, leftVolts, voltageUnits::volt);
//   //   bl.spin(forward, leftVolts, voltageUnits::volt);
//   //   fr.spin(forward, rightVolts, voltageUnits::volt);
//   //   mr.spin(forward, rightVolts, voltageUnits::volt);
//   //   br.spin(forward, rightVolts, voltageUnits::volt);



