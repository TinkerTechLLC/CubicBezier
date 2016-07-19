#include <CubicBezier.h>
#include <hermite_spline.h>
#include <key_frames\key_frames.h>

// Variables
long start_time;
int trials = 1;
int time_steps = 1000;
OrderedPair ctrl_pts[4];

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

	hornerTest();
	hermiteTest();	
	hornerIncremental();
}

void hornerIncremental(){
	CubicBezier bezier = CubicBezier();
	bezier.knotCount(3);
	bezier.setNextX(0);
	bezier.setNextX(25);
	bezier.setNextX(50);
	bezier.setNextX(75);
	
	bezier.setNextX(100);
	bezier.setNextX(125);
	bezier.setNextX(150);
	bezier.setNextX(175);
	
	bezier.setNextY(0);
	bezier.setNextY(60);
	bezier.setNextY(10);
	bezier.setNextY(75);
	
	
	bezier.setNextY(100);
	bezier.setNextY(50);
	bezier.setNextY(30);
	bezier.setNextY(15);
	bezier.initSpans();

	USBSerial.println("\nCubic Bezier control points after incremental setup");
	printSpanCtrlPts(bezier.getSpan(0));
	printSpanCtrlPts(bezier.getSpan(1));

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

	USBSerial.print("Optimized execution time 2: ");
	USBSerial.println(optimized_time);
}

void hornerTest(){
	USBSerial.println("\nInitializing Cubic Bezier");
	CubicBezier bezier = CubicBezier(ctrl_pts, 2, false);
	bezier.initSpans();

	USBSerial.println("\nVerifying Bezier spline control points");
	Span *first_span = bezier.getSpan(0);
	printSpanCtrlPts(first_span);

	USBSerial.println("\nGetting spline values with Horner's rule method");

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

void hermiteTest(){
	KeyFrames kf = KeyFrames();
	kf.setKFCount(2);
	kf.setXN((float)0);
	kf.setXN(25);
	kf.setXN(50);
	kf.setXN(75);
	int count = 4;
	for (int i = 0; i < count; i++){
		kf.setDN((float)0);
	}

	float increment = 75 / time_steps;
	startTimer();
	for (int j = 0; j < trials; j++){
		for (int i = 0; i < time_steps + 1; i++){
			float x = increment * (float)i;
			kf.pos(x);
		}
	}
	long hermite_time = elapsedMicros();

	USBSerial.print("Hermite execution time: ");
	USBSerial.println(hermite_time);
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


