#include <CubicBezier.h>

// Variables
long start_time;

void setup(){		
	Serial.begin(9600);
	printMemoryConsumption();
	timeTrial();	
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

void timeTrial(){

	Serial.println("\nSetting control points");	
	OrderedPair ctrl_pts[4];
	ctrl_pts[0] = OrderedPair(0, 0);
	ctrl_pts[1] = OrderedPair(25, 60);
	ctrl_pts[2] = OrderedPair(50, 10);
	ctrl_pts[3] = OrderedPair(75, 75);

	const int CTRL_PT_COUNT = 4;
	for (int i = 0; i < CTRL_PT_COUNT; i++){
		Serial.print(i);
		Serial.print(" -- x: ");
		Serial.print(ctrl_pts[i].x());
		Serial.print(" y: ");
		Serial.println(ctrl_pts[i].y());
	}
	
	Serial.println("\nInitializing Cubic Bezier");
	CubicBezier bezier = CubicBezier(ctrl_pts, 2, false);
	bezier.initSpans();

	Serial.println("\nVerifying Bezier spline control points");
	Span *first_span = bezier.getSpan(0);
	printSpanCtrlPts(first_span);

	int trials = 1;
	Serial.println("\nGetting spline values with naive method");
	int time_steps = 1000;
	float h = (float)1 / time_steps;
	startTimer();
	for (int j = 0; j < trials; j++){
		for (int i = 0; i < time_steps + 1; i++){
			float t = i * h;
			OrderedPair point = bezier.positionAtT(t);
			//cout << "t: "<< t << " x: " << point.x() << " y: "<< point.y() << endl;
		}
	}
	long naive_time = elapsedMicros();
	Serial.print("Naive execution time: ");
	Serial.println(naive_time);

	Serial.println("\nGetting spline values with optimized method");

	bezier.initForwardDiff(h);
	startTimer();
	for (int j = 0; j < trials; j++){		
		for (int i = 0; i < time_steps; i++){
			OrderedPair point = bezier.positionAtNextT();
			//cout << "t: " << (i+1)*h << " x: " << point.x() << " y: " << point.y() << endl;
		}
	}
	long optimized_time = elapsedMicros();

	Serial.print("Optimized execution time: ");
	Serial.println(optimized_time);
	Serial.print("Optimized %: ");
	Serial.println((float)optimized_time / (float)naive_time);
}

void startTimer(){
	start_time = micros();
}
long elapsedMicros(){
	long elapsed = micros() -start_time;
	return elapsed;
}
void printSpanCtrlPts(Span *this_span){	 
	Serial.print("Span ID: ");
	Serial.println(this_span->id());
	for (int i = 0; i < 4; i++){
		OrderedPair *pts = this_span->ctrlPts();
		Serial.print("ctrl_pt ");
		Serial.print(i);
		Serial.print(" Address: ");		
		Serial.print((int)&pts[i], HEX);
		Serial.print(" x: ");
		Serial.print(pts[i].x());
		Serial.print(" y: ");
		Serial.println(pts[i].y());
	}	
}


