// digital pin 10 is the hall pin
int hall_pin = 2; //be careful to set to a pin capable of generating interrupts
int hallState;
// set number of hall trips for RPM reading (higher improves accuracy)
int hall_thresh = 5;
int hall_count = 1;
//time the program starts
  float start = micros();

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // make the hall pin an input:
  pinMode(hall_pin, INPUT);
  attachInterrupt(0,increment, RISING); //interrupts when hall_pin is on the rise 
  // change interrupt pin based on the hall_pin (arduino uno (2 -> 0 and 3 -> 1)
}

// the loop routine runs over and over again forever:
void loop() {


  //Serial.print("23 \n");
  //starts counting again after a certain threshold value for hall_count  
  if (hall_count>=hall_thresh){
     // print information about Time and RPM
     float time_passed = ((micros()-start)/1000000.0);
     Serial.print("Time Passed: ");
     Serial.print(time_passed);
     Serial.println("s");
     
     float rpm_val = (hall_count/time_passed)*60.0;
     Serial.print(rpm_val);
     Serial.println(" RPM");

     //reset hall_count to initial value
     hall_count = 1;

     start = micros();
    }
  
}

// increments hall_count when system is interrupted
void increment(){
  hall_count++;
}
