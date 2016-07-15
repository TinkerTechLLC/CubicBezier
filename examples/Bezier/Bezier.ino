#include <CubicBezier.h>

// Variables
long start_time;

void setup(){		
	USBSerial.begin(9600);
	while (!USBSerial){}
	printMemoryConsumption();
	timeTrial();	
}

void loop(){

}

void printMemoryConsumption(){
	USBSerial.print("Cubic Bezier memory size: ");
	USBSerial.println(sizeof(CubicBezier));
	USBSerial.print("Span memory size: ");
	USBSerial.println(sizeof(Span));
	USBSerial.print("OrderedPair memory size: ");
	USBSerial.println(sizeof(OrderedPair));
}

void timeTrial(){

	USBSerial.println("\nSetting control points");	
	OrderedPair ctrl_pts[4];
	ctrl_pts[0] = OrderedPair(0, 0);
	ctrl_pts[1] = OrderedPair(25, 60);
	ctrl_pts[2] = OrderedPair(50, 10);
	ctrl_pts[3] = OrderedPair(75, 75);

	const int CTRL_PT_COUNT = 4;
	for (int i = 0; i < CTRL_PT_COUNT; i++){
		USBSerial.print(i);
		USBSerial.print(" -- x: ");
		USBSerial.print(ctrl_pts[i].x());
		USBSerial.print(" y: ");
		USBSerial.println(ctrl_pts[i].y());
	}
	
	USBSerial.println("\nInitializing Cubic Bezier");
	CubicBezier bezier = CubicBezier(ctrl_pts, 2, false);
	bezier.initSpans();

	USBSerial.println("\nVerifying Bezier spline control points");
	Span *first_span = bezier.getSpan(0);
	printSpanCtrlPts(first_span);

	int trials = 1;
	USBSerial.println("\nGetting spline values with Horner's rule method");
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
	long optimized_time = elapsedMicros();

	USBSerial.print("Optimized execution time: ");
	USBSerial.println(optimized_time);
}

void startTimer(){
	start_time = micros();
}
long elapsedMicros(){
	long elapsed = micros() -start_time;
	return elapsed;
}
void printSpanCtrlPts(Span *this_span){	 
	USBSerial.print("Span ID: ");
	USBSerial.println(this_span->id());
	for (int i = 0; i < 4; i++){
		OrderedPair *pts = this_span->ctrlPts();
		USBSerial.print("ctrl_pt ");
		USBSerial.print(i);
		USBSerial.print(" Address: ");		
		USBSerial.print((int)&pts[i], HEX);
		USBSerial.print(" x: ");
		USBSerial.print(pts[i].x());
		USBSerial.print(" y: ");
		USBSerial.println(pts[i].y());
	}	
}


