int sensor1 = A0;      
int sensor2 = A1;
int sensor3 = A2;
int sensor4 = A3;  

int sensor[4] = {0, 0, 0, 0};

int ENA = 9;
int m1 = 8; //right        
int m2 = 7;
int m3 = 5;   //left
int m4 = 4;
int ENB = 3;

int initial_motor_speed = 120;

float Kp = 25;
float Ki = 0;
float Kd = 15;

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;

int flag = 0;

void setup()
{
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  

  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(m3, OUTPUT);
  pinMode(m4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  Serial.begin(9600);                     
  delay(500);
  Serial.println("Start");
  delay(1000);
}
void loop()
{
  read();
  Serial.print(error);
  if (error == 100) {               // Make left turn untill it detects straight path

    do {
      read();
      analogWrite(ENA, 60); 
      analogWrite(ENB, 60); 
      sharpLeftTurn();
    } while (error != 0);

  } else if (error == 101) {          // Make right turn in case of it detects only right path (it will go into forward direction in case of straight and right "|--")
                                      // untill it detects straight path.
    analogWrite(ENA, 60); 
    analogWrite(ENB, 60); 
    forward();
    delay(200);
    stop();
    read();
    if (error == 102) {
      do {
        analogWrite(ENA, 60); 
        analogWrite(ENB, 60); 
        sharpRightTurn();
        read();
      } while (error != 0);
    }
  } else if (error == 102) {        // Make left turn untill it detects straight path
  
    do {
      analogWrite(ENA, 60);
      analogWrite(ENB, 60); 
      sharpLeftTurn();
      read();
      if (error == 0) {
        stop();
        delay(200);
      }
    } while (error != 0);
  } else if (error == 103) {        // Make left turn untill it detects straight path or stop if dead end reached.
    if (flag == 0) {
      analogWrite(ENA, 60); //Left Motor Speed
      analogWrite(ENB, 60); //Right Motor Speed
      forward();
      delay(200);
      stop();
      read();
      if (error == 103) {     /**** Dead End Reached, Stop! ****/
        stop();
        flag = 1;
      } else {        /**** Move Left ****/
        analogWrite(ENA, 60); //Left Motor Speed
        analogWrite(ENB, 60); //Right Motor Speed
        sharpLeftTurn();
        delay(200);
        do {
          //Serial.print("\t");
          //Serial.println("Left Here");
          read();
          analogWrite(ENA, 60); //Left Motor Speed
          analogWrite(ENB, 60); //Right Motor Speed
          sharpLeftTurn();
        } while (error != 0);
      }
    }
  } else {
    calculate_pid();
    motor_control();
  }
}

void read()
{
  sensor[0] = digitalRead(sensor1);
  sensor[1] = digitalRead(sensor2);
  sensor[2] = digitalRead(sensor3);
  sensor[3] = digitalRead(sensor4);

  if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0))
    error = 3;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0))
    error = 2;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0))
    error = 1;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0))
    error = 0;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0))
    error = -1;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1))
    error = -2;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1))
    error = -3;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0)) // Turn robot left side
    error = 100;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1)) // Turn robot right side
    error = 101;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0)) // Make U turn
    error = 102;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1)) // Turn left side or stop   go forward if all black then stop , go forward no black turn left
    error = 103;
}

void calculate_pid()
{
  P = error;
  I = I + previous_I;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error;
}

void motor_control()
{
  // Calculating the effective motor speed:
  int left_motor_speed = initial_motor_speed - PID_value;
  int right_motor_speed = initial_motor_speed + PID_value;

  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);


  analogWrite(ENA, left_motor_speed);
  analogWrite(ENB, right_motor_speed); 

  //following lines of code are to make the bot move forward
  forward();
}

void forward()
{
  
  digitalWrite(m1, LOW);
  digitalWrite(m2, HIGH);
  digitalWrite(m3, LOW);
  digitalWrite(m4, HIGH);
}
void reverse()
{
  
  digitalWrite(m1, HIGH);
  digitalWrite(m2, LOW);
  digitalWrite(m3, HIGH);
  digitalWrite(m4, LOW);
}
void right()
{
  digitalWrite(m1, LOW);
  digitalWrite(m2, HIGH);
  digitalWrite(m3, LOW);
  digitalWrite(m4, LOW);
}
void left()
{
  digitalWrite(m1, LOW);
  digitalWrite(m2, LOW);
  digitalWrite(m3, LOW);
  digitalWrite(m4, HIGH);
}
void sharpRightTurn() 
{
  digitalWrite(m1, LOW);
  digitalWrite(m2, HIGH);
  digitalWrite(m3, HIGH);
  digitalWrite(m4, LOW);
}
void sharpLeftTurn() 
{
  digitalWrite(m1, HIGH);
  digitalWrite(m2, LOW);
  digitalWrite(m3, LOW);
  digitalWrite(m4, HIGH);   
}
void stop()
{
  digitalWrite(m1, LOW);
  digitalWrite(m2, LOW);
  digitalWrite(m3, LOW);
  digitalWrite(m4, LOW);
}
