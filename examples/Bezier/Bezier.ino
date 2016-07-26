#include <CubicBezier.h>
#include <hermite_spline.h>

// Variables
long start_time;
int trials = 1;
int time_steps = 1000;
OrderedPair ctrl_pts[4];

void setup(){		
	Serial.begin(9600);
	while (!Serial){}
	printMemoryConsumption();


        CubicBezier bezier = CubicBezier();
        bezier.knotCount(2);
        bezier.setNextX(0);
        bezier.setNextX(10);
        bezier.setNextX(11);
        bezier.setNextX(75);
      	
        bezier.setNextY(0);
        bezier.setNextY(60);
        bezier.setNextY(10);
        bezier.setNextY(75);
      
        bezier.initSpans();

        const int INC_COUNT = 20;
        float hx = 75.0 / INC_COUNT;
        float results[INC_COUNT+1];
        startTimer();
        for(int i = 0; i < INC_COUNT+1; i++){
          results[i] = bezier.positionAtX(hx * i);
        }
        long optimized_time = elapsedMicros();
        Serial.print("Optimized execution time: ");
        Serial.println(optimized_time);
        for(int i = 0; i < INC_COUNT+1; i++){
          Serial.println(results[i]);
        }
}

void loop(){

}

void printMemoryConsumption(){
	Serial.print("Cubic Bezier memory size: ");
	Serial.println(sizeof(CubicBezier));
	Serial.print("Span memory size: ");
	Serial.println(sizeof(Span));
	Serial.print("OrderedPair memory size: ");
	Serial.println(sizeof(OrderedPair));
}

void startTimer(){
	start_time = micros();
}
long elapsedMicros(){
	long elapsed = micros() -start_time;
	return elapsed;
}


