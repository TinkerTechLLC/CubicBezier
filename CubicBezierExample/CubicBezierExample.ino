#include <CubicBezier.h>
#define OP OrderedPair
#define P USBSerial.print
#define PLN USBSerial.println
OP ctrl_pts[4];
CubicBezier bezier;
const int CTRL_PT_COUNT = 4;

void setup(){

    USBSerial.begin(9600);

    while(!USBSerial){}
    PLN("Setting control points");
    ctrl_pts[0] = OP(0,  0);
    ctrl_pts[1] = OP(25, 50);
    ctrl_pts[2] = OP(50, 50);
    ctrl_pts[3] = OP(75, 0);
    
    for(int i = 0; i < CTRL_PT_COUNT; i++){
      P(i);
      P(" -- x: ");
      P(ctrl_pts[i].x());
      P(" y: ");
      PLN(ctrl_pts[i].y());
    }

    PLN("\nInitializing Cubic Bezier");
    bezier = CubicBezier(ctrl_pts, 2, false);
    
    PLN("\nPrinting first span points");
    bezier.printFirstSpanPts();
   
    PLN("\nGetting spline values");
    int time_steps = 4;
    float h = (float)1/time_steps;
    for(int i = 0; i < time_steps+1; i++){
        float t = i * h;
        OP point = bezier.positionAtT(t);
        P("t: ");
        P(t);
        P(" x: ");
        P(point.x());
        P(" y: ");
        PLN(point.y());
    }
}

void loop(){

}
