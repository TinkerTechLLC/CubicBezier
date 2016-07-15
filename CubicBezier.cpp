#include "stdafx.h"
#include "CubicBezier.h"


#if defined(__AVR_AT90USB1287__)
#define SERIAL USBSerial
#define EMBEDDED
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)  || defined(__AVR_ATmega168__)
#define SERIAL Serial
#define EMBEDDED
#else
#define DESKTOP
#endif

#if defined(EMBEDDED)
// Defines
#define P SERIAL.print
#define PLN SERIAL.println
#define STRING String
bool CubicBezier::g_embedded = true;
#elif defined(DESKTOP)
#include <iostream>
#include <iomanip>
bool CubicBezier::g_embedded = false;
using std::cout;
using std::endl;
using std::hex;
using std::dec;
#endif



/*
	This library provides the ability to compute cubic Bezier curves.
	An optimized forward differencing method may be used when values
	along the curve are desired in fixed t parameter increments.
*/

/********** OrderedPair class functions **********/

#pragma region OrderedPair Class Functions

int OrderedPair::g_id_gen = 0;

OrderedPair::OrderedPair(){}

OrderedPair::OrderedPair(float p_x, float p_y){
	m_id = g_id_gen++;
	m_vals[0] = p_x;
	m_vals[1] = p_y;
}

float OrderedPair::val(int p_which){
	// Enforce a valid request
	p_which = p_which < 0 ? 0 : p_which > 1 ? 1 : p_which;
	return m_vals[p_which];
}

void OrderedPair::val(int p_which, float p_val){	
	m_vals[p_which] = p_val;
}

float  OrderedPair::x(){
	return m_vals[0];
}

float OrderedPair::y(){
	return m_vals[1];
}


#pragma region Arithmatic Operator Overrides
OrderedPair operator * (int p_x, const OrderedPair& p_op){
	return OrderedPair(p_op.m_vals[0] * p_x, p_op.m_vals[1] * p_x);
}
OrderedPair operator * (const OrderedPair& p_op, int p_x){
	return OrderedPair(p_op.m_vals[0] * p_x, p_op.m_vals[1] * p_x);
}
OrderedPair operator * (float p_x, const OrderedPair& p_op){
	return OrderedPair(p_op.m_vals[0] * p_x, p_op.m_vals[1] * p_x);
}
OrderedPair operator * (const OrderedPair& p_op, float p_x){
	return OrderedPair(p_op.m_vals[0] * p_x, p_op.m_vals[1] * p_x);
}
OrderedPair operator + (int p_x, const OrderedPair& p_op){
	return OrderedPair(p_op.m_vals[0] + p_x, p_op.m_vals[1] + p_x);
}
OrderedPair operator + (float p_x, const OrderedPair& p_op){
	return OrderedPair(p_op.m_vals[0] + p_x, p_op.m_vals[1] + p_x);
}
OrderedPair operator + (const OrderedPair& p_op, int p_x){
	return OrderedPair(p_op.m_vals[0] + p_x, p_op.m_vals[1] + p_x);
}
OrderedPair operator + (const OrderedPair& p_op, float p_x){
	return OrderedPair(p_op.m_vals[0] + p_x, p_op.m_vals[1] + p_x);
}
OrderedPair operator + (const OrderedPair& p_op1, const OrderedPair& p_op2){
	return OrderedPair(p_op1.m_vals[0] + p_op2.m_vals[0], p_op1.m_vals[1] + p_op2.m_vals[1]);
}
OrderedPair operator - (int p_x, const OrderedPair& p_op){
	return OrderedPair(p_op.m_vals[0] - p_x, p_op.m_vals[1] * p_x);
}
OrderedPair operator - (const OrderedPair& p_op){
	return OrderedPair(-p_op.m_vals[0], -p_op.m_vals[1]);
}
#pragma endregion

#pragma endregion

/********** Span class functions **********/

#pragma region Span Class Functions

int Span::g_id_gen = 0;

Span::Span(){}

Span::Span(OrderedPair *p_ctrl_pts, Span *p_prev_span){		
	m_id = g_id_gen++;
	m_h = NULL;
	m_T = NULL;
	m_t_steps_remain = NULL;
	for (int i = 0; i < g_MAX_CTRL_PTS; i++){
		m_ctrl_pts[i] = p_ctrl_pts[i];
	}	
	m_next_span = NULL;
	prevSpan(p_prev_span);			
	setCoeffs();	
}

void Span::setCoeffs(){	
	m_coeff_A = (-m_ctrl_pts[0]) + (3 * m_ctrl_pts[1]) + (-3 * m_ctrl_pts[2]) + m_ctrl_pts[3];
	m_coeff_B = (3 * m_ctrl_pts[0]) + (-6 * m_ctrl_pts[1]) + (3 * m_ctrl_pts[2]);
	m_coeff_C = (-3 * m_ctrl_pts[0]) + (3 * m_ctrl_pts[1]);
	m_coeff_D = m_ctrl_pts[0];
}

void Span::nextSpan(Span *p_next_span){	
	m_next_span = p_next_span;
}

Span* Span::nextSpan(){
	return m_next_span;
}

void Span::prevSpan(Span *p_prev_span){	
	m_prev_span = p_prev_span;	
	if (m_prev_span != NULL)
		m_prev_span->nextSpan(this);
}

Span* Span::prevSpan(){
	return m_prev_span;
}

/**
	@return The number of steps in this segment
*/
void Span::incrementSizeFromX(float p_x){
	// Find h as x fraction of total span X range
	float h = p_x / rangeX();
	incrementSize(h);
	m_t_steps_remain = (int)round(1 / h);
	//cout << "Span steps remaining: " << m_t_steps_remain << endl;
}

void Span::incrementSize(float p_h){
	m_h = p_h;
	initForwardDiff();
}

float Span::incrementSize(){
	return m_h;
}

void Span::initForwardDiff(){
	// Set the initial forward differences	
	m_fdiff_vals[0] = m_coeff_D;
	m_fdiff_vals[1] = m_coeff_A * pow(m_h, 3) + m_coeff_B * pow(m_h, 2) + m_coeff_C * m_h;
	m_fdiff_vals[3] = 6 * m_coeff_A * pow(m_h, 3);
	m_fdiff_vals[2] = m_fdiff_vals[3] + (2 * m_coeff_B * pow(m_h, 2));	

	/*cout << "h: " << m_h << "h^3: " << pow(m_h, 3) << "\n" << endl;

	cout << "Coeffs:" << endl;
	cout << "Ax: " << m_coeff_A.x() << " Bx: " << m_coeff_B.x() << " Cx: " << m_coeff_C.x() << " Dx: " << m_coeff_D.x() << endl;
	cout << "Ay: " << m_coeff_A.y() << " By: " << m_coeff_B.y() << " Cy: " << m_coeff_C.y() << " Dy: " << m_coeff_D.y() << "\n" << endl;

	cout << "Setting initial F-diffs" << endl;
	cout << "Px: " << m_fdiff_vals[0].x() << " F1x: " << m_fdiff_vals[1].x() << " F2x: " << m_fdiff_vals[2].x() << " F3x: " << m_fdiff_vals[3].x() << endl;
	cout << "Py: " << m_fdiff_vals[0].y() << " F1y: " << m_fdiff_vals[1].y() << " F2y: " << m_fdiff_vals[2].y() << " F3y: " << m_fdiff_vals[3].y() << "\n" << endl;
	*/

	// Zero the t step counter
	m_T = 0;
}

int Span::stepsRemaining(){
	return m_t_steps_remain;
}
	
OrderedPair Span::positionAtNextT(){
	//cout << "Px: " << m_fdiff_vals[0].x() << " F1x: " << m_fdiff_vals[1].x() << " F2x: " << m_fdiff_vals[2].x() << " F3x: " << m_fdiff_vals[3].x() << endl;
	//cout << "Py: " << m_fdiff_vals[0].y() << " F1y: " << m_fdiff_vals[1].y() << " F2y: " << m_fdiff_vals[2].y() << " F3y: " << m_fdiff_vals[3].y() << "\n" << endl;
	const int FORWARD_DIFFS = 3;
	for (int i = 0; i < FORWARD_DIFFS; i++){
		m_fdiff_vals[i] = m_fdiff_vals[i] + m_fdiff_vals[i + 1];
	}
	m_T += m_h;
	m_t_steps_remain--;
	return m_fdiff_vals[0];
}

OrderedPair Span::positionAtT(float p_T){	
	float pos[2];
	for (int i = 0; i < 2; i++){
		// B(t) = At^3 + Bt^2 + Ct + D
		pos[i] = m_coeff_A.val(i) * pow(p_T, 3) + m_coeff_B.val(i) * pow(p_T, 2) + m_coeff_C.val(i) * p_T + m_coeff_D.val(i);
	}
	return OrderedPair(pos[0], pos[1]);
}

bool Span::containsX(float p_x){	
	if (p_x >= minX() && p_x <= maxX())	
		return true;
	else		
		return false;
}

bool Span::containsY(float p_y){
	if (minY() <= p_y && p_y <= maxY())
		return true;
	else
		return false;
}

float Span::minX(){		
	return m_ctrl_pts[0].x();
}

float Span::maxX(){
	return m_ctrl_pts[g_MAX_CTRL_PTS-1].x();
}

float Span::rangeX(){
	return maxX() - minX();
}

float Span::minY(){
	return m_ctrl_pts[0].y();
}

float Span::maxY(){
	return m_ctrl_pts[g_MAX_CTRL_PTS-1].y();
}

float Span::rangeY(){
	return maxY() - minY();
}

#if defined DESKTOP
void Span::printCtrlPts(){
	cout << "Span ID: " << id() << endl;
	for(int i = 0; i < 4; i++){
		cout << "ctrl_pt " << i << " Address: " << hex << (int)&m_ctrl_pts[i] 
			<< dec << " x: " << m_ctrl_pts[i].x() << " y: " << m_ctrl_pts[i].y() << endl;
	}
	cout << "Done printing points" << endl;
}
#elif defined EMBEDDED
void Span::printCtrlPts(){
	P("Span ID: ");
	PLN(id());
	for (int i = 0; i < 4; i++){
		P("ctrl_pt ");
		P(i);
		P(" Address: ");
		P((int)&m_ctrl_pts[i], hex);
		P(" x: ");
		P(m_ctrl_pts[i].x());
		P(" y: ");
		PLN(m_ctrl_pts[i].y());
	}
	PLN("Done printing points");
}
#endif


int Span::id(){
	return m_id;
}


#pragma endregion

/********** CubicBezier class functions **********/

#pragma region CubicBezier Class Functions

CubicBezier::CubicBezier(){}

/*
	If specifying only knots, inner control points are set equal
	to their nearest knot, i.e.	p1 = p0 and p2 = p3, resulting
	in a linear spline. Control points p1 and p2 for each span
	may be modified later.
*/
CubicBezier::CubicBezier(OrderedPair *p_ctrl_pts, int p_knot_count, bool p_only_knots){
	// Must have at least 2 knots to define a spline	
	if (p_knot_count < 2)
		return;
	m_ctrl_pts = p_ctrl_pts;	
	m_knot_count = p_knot_count;
	m_span_count = p_knot_count - 1;
	m_mem_allocated = false;
	m_only_knots = p_only_knots;			
}

// Default destructor
CubicBezier::~CubicBezier(){
	releaseMemory();
}

void CubicBezier::releaseMemory(){
	if (m_mem_allocated)
		free(m_spans);
}

void CubicBezier::initSpans(){
	
	cout << "Bez - Initializing spans" << endl;
	// Dynamically allocate memory for span array
	if (m_mem_allocated){	
		cout << "Bez - Releasing allocated memory" << endl;
		releaseMemory();
	}
	size_t span_size = sizeof(Span);	
	cout << "Bez - Span size: " << span_size << endl;	
	size_t memory_needed = m_span_count * span_size;	
	cout << "Bez - Spans: " << m_span_count << endl;
	cout << "Bez - Memory needed: " << memory_needed << endl;
	cout << "Bez - Allocating memory for span array" << endl;
	m_spans = (Span *)malloc(memory_needed);
	m_mem_allocated = true;

	/** Extract control points for each span and create new span objects **/

	/* 
		Adjust the element being accessed, depending on whether only knots 
		have been included. If only knots are used, set the inner control
		points equal to their nearest knot.
	*/
	int adjust = m_only_knots ? 0 : 1;		
	int PT_CT = 4;
	for (int i = 0; i < m_span_count; i++){				
		OrderedPair ctrl_pt_subset[4] = { m_ctrl_pts[0 + i * PT_CT], m_ctrl_pts[0 + i * PT_CT + adjust], 
			m_ctrl_pts[1 + i * PT_CT + adjust], m_ctrl_pts[1 + i * PT_CT + 2 * adjust] };
		// Don't pass a garbage address to the pointer parameter for the first span
		Span* prev_span = i == 0 ? NULL : &m_spans[i - 1];				
		m_spans[i] = Span(ctrl_pt_subset, prev_span);		
	}		
	m_cur_span = &m_spans[0];			
}

OrderedPair CubicBezier::positionAtT(float p_T){

	float max_val;
	if (m_only_knots)
		max_val = m_ctrl_pts[m_knot_count - 1].x();
	else
		max_val = m_ctrl_pts[m_knot_count * 2 - 1].x();
	
	// Parameter t as x position
	p_T = p_T * max_val;

	// Determine which span to check
	for (int i = 0; i < m_span_count; i++){
		if (m_spans[i].containsX(p_T)){
			// Convert parameter to t of given span and get result
			//cout << "minX: " << m_spans[i].minX() << " rangeX: " << m_spans[i].rangeX() << " p_T: " << p_T << "\n" << endl;
			p_T = (p_T - m_spans[i].minX()) / m_spans[i].rangeX();
			return m_spans[i].positionAtT(p_T);
		}
	}
	return OrderedPair(0, 0);
}

void CubicBezier::printFirstSpanPts(){
	m_spans[0].printCtrlPts();
}

OrderedPair CubicBezier::positionAtNextT(){
	
	// Get the next position
	OrderedPair ret  = m_cur_span->positionAtNextT();

	/* 
		Stop before the final t-step for all spans except the final one,
		since the beginning of the next span will be the same as the end
		of the previous one.
	*/
	int end_count = 1;
	if (m_cur_span->nextSpan() == NULL){		
		end_count = 0;
	}

	// Move on to the next span if necessary
	if (m_cur_span->stepsRemaining() < end_count){		
		m_cur_span = m_cur_span->nextSpan();
	}

	m_t_steps_remain--;

	return ret;
}

void CubicBezier::initForwardDiff(float p_h){
	m_t_steps_remain = (int)(1 / p_h);
	p_h *= m_spans[m_span_count - 1].maxX();

	// Set the increment sizes for each of the segments
	for (int i = 0; i < m_span_count; i++){
		m_spans[i].incrementSizeFromX(p_h);
	}
}

float CubicBezier::incrementSize(){
	return m_h;
}

int CubicBezier::stepsRemaining(){
	return m_t_steps_remain;
}

#pragma endregion
