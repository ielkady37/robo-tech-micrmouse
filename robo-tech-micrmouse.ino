//#include "Robot.h"
//
//
//Robot robot;
//void setup() {
//  Serial.begin(115200);
//  delay(1500);
//  robot.begin();
//  delay(500);
//  Serial.println("Ready!");
//  pinMode(2, OUTPUT);
//}
//
//void loop() {
//  delay(1000);
//  robot.turn(90);
//   robot.move(1);
//   robot.move(1);
//   robot.move(1);
//   robot.move(1);
//  robot.turn(-90);
//  
//   robot.move(1);
//   robot.move(1);
//   robot.move(1);
//   robot.move(1);
//   robot.move(1);
//   robot.move(1);
//   robot.turn(-90);
//  
//   robot.move(1);
//   robot.move(1);
//   robot.move(1);
//   robot.move(1);
//   robot.turn(-90);
//
//   robot.move(1);
//   robot.move(1);
//   robot.move(1);
//   robot.move(1);
//   robot.move(1);
//   robot.turn(-90);
//  
//   robot.move(1);
//   robot.turn(-90);
//  
//   robot.move(1);
//   robot.move(1);
//   robot.move(1);
//   robot.move(1);
//   robot.turn(90);
//  
//   robot.move(1);
//   robot.move(1);
//   robot.turn(90);
//  
//   robot.move(1);
//  // // robot.print_all_sensors();
//  digitalWrite(2, HIGH);
//  delay(1000);
//  digitalWrite(2, LOW);
//}


 #include "API.h"
 #include "algorithm.h"
 int solveSwitch = 12; //yarab mnsash a8irha
 API api;
 void setup() {
 //  pinMode(2, OUTPUT);
   pinMode(solveSwitch,INPUT_PULLUP);
   api.begin();
   if(digitalRead(solveSwitch)){
     Serial.print("ht2oly ana wana wl7dyd etana");
     loadMatrix();
   }
  
 }

 void loop() {
    floodFill(api);
 //  delay(1000);
 //  api.turnLeft();
 //  api.printSensors();
 //  digitalWrite(2, HIGH);
 //  delay(1000);
 //  digitalWrite(2, LOW);
   // Serial.print("|| Front Wall: ");
   // Serial.print(api.wallFront());
   // Serial.print("|| Right Wall: ");
   // Serial.print(api.wallRight());
   // Serial.print("|| Left Wall: ");
   // Serial.println(api.wallLeft());
 }
