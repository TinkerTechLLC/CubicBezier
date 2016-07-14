# include "CubicBezier.h"

/*

	This library provides the ability to compute cubic bezier curves

*/

/********** OrderedPair class functions **********/

#pragma region OrderedPair Class Functions
OrderedPair::OrderedPair(){}

OrderedPair::OrderedPair(float p_x, float p_y){
	val(0, p_x);			
	val(1, p_y);
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
OrderedPair operator + (int p_x, const OrderedPair& p_op){
	return OrderedPair(p_op.m_vals[0] + p_x, p_op.m_vals[1] * p_x);
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
	USBSerial.println("Span - Constructor");	
	m_ctrl_pts = p_ctrl_pts;
	m_next_span = NULL;
	prevSpan(p_prev_span);	
	if(m_prev_span != NULL)
		m_prev_span->nextSpan(this);	
	setCoeffs();
	USBSerial.println("Span - Done constructing");
}

void Span::setCoeffs(){
	USBSerial.println("Span - Setting coeffs");
	m_coeffs[0] = (-m_ctrl_pts[0]) + (3 * m_ctrl_pts[1]) + (-3 * m_ctrl_pts[2]) + m_ctrl_pts[3];
	m_coeffs[1] = (3 * m_ctrl_pts[0]) + (-6 * m_ctrl_pts[1]) + (3 * m_ctrl_pts[2]);
	m_coeffs[2] = (-3 * m_ctrl_pts[0]) + (3 * m_ctrl_pts[1]);
	m_coeffs[3] = m_ctrl_pts[0];
}

void Span::nextSpan(Span *p_next_span){
	USBSerial.println("Span - Setting next span");
	m_next_span = p_next_span;
}

Span* Span::nextSpan(){
	return m_next_span;
}

void Span::prevSpan(Span *p_prev_span){
	USBSerial.println("Span - Setting prev span");
	m_prev_span = p_prev_span;	
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
	USBSerial.println("Span - Initializing forward differences");
	m_fdiff_vals[0] = m_coeffs[3];
	m_fdiff_vals[1] = m_coeffs[0] * pow(m_h, 3);
	m_fdiff_vals[2] = m_coeffs[1] * pow(m_h, 2);
	m_fdiff_vals[3] = m_coeffs[2] * m_h;

	// Zero the t step counter
	m_T = 0;
}

int Span::stepsRemaining(){
	return m_t_steps_remain;
}
	
OrderedPair Span::positionAtNextT(){
	const int FORWARD_DIFFS = 3;
	for (int i = 0; i < FORWARD_DIFFS; i++){
		m_fdiff_vals[i] + m_fdiff_vals[i + 1];
	}
	m_T += m_h;
	m_t_steps_remain--;
	return m_fdiff_vals[0];
}

OrderedPair Span::positionAtT(float p_T){
	USBSerial.print("Span - Getting position at t: ");
	USBSerial.println(p_T);
	float pos[2];
	for (int i = 0; i < 2; i++){
		// B(t) = At^3 + Bt^2 + Ct + D
		pos[i] = m_coeffs[0].val(i) * pow(p_T, 3) + m_coeffs[1].val(i) * pow(p_T, 2) + m_coeffs[2].val(i) * p_T + m_coeffs[3].val(i);
	}
	return OrderedPair(pos[0], pos[1]);
}

boolean Span::containsX(float p_x){
	USBSerial.print("Spline contains x val ");
	USBSerial.print(p_x);
	USBSerial.print("? ");
	if (p_x >= minX() && p_x <= maxX()){
		USBSerial.println("True");
		return true;
	}
	else{
		USBSerial.println("False");
		return false;
	}
}

boolean Span::containsY(float p_y){
	if (minY() <= p_y && p_y <= maxY())
		return true;
	else
		return false;
}

float Span::minX(){	
	USBSerial.print("minX: ");
	USBSerial.println(m_ctrl_pts[0].x());
	return m_ctrl_pts[0].x();
}

float Span::maxX(){
	USBSerial.print("maxX: ");
	USBSerial.println(m_ctrl_pts[3].x());
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

void Span::printCtrlPts(){
	for(int i = 0; i < 4; i++){
		USBSerial.print("ctrl_pt ");
		USBSerial.print(i);
		USBSerial.print(" x: ");
		USBSerial.print(m_ctrl_pts[i].x());
		USBSerial.print(" y: ");
		USBSerial.println(m_ctrl_pts[i].y());
	}
	USBSerial.println("Done printing points");
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
CubicBezier::CubicBezier(OrderedPair *p_ctrl_pts, int p_knot_count, boolean p_only_knots){
	// Must have at least 2 knots to define a spline
	USBSerial.println("Bez - Constructor");
	if (p_knot_count < 2)
		return;
	m_ctrl_pts = p_ctrl_pts;	
	m_knot_count = p_knot_count;
	m_span_count = p_knot_count - 1;
	m_mem_allocated = false;
	m_only_knots = p_only_knots;
	USBSerial.println("Bez - Pre-initalization");
	m_spans[0].printCtrlPts();
	initializeSpans();
	USBSerial.println("Bez - Post-initalization");
	m_spans[0].printCtrlPts();
	USBSerial.println("Bez - Done constructing");
}

// Default destructor
CubicBezier::~CubicBezier(){
	releaseMemory();
}

void CubicBezier::releaseMemory(){
	if (m_mem_allocated)
		free(m_spans);
}

void CubicBezier::initializeSpans(){
	USBSerial.println("Bez - Initializing spans");
	// Dynamically allocate memory for span array
	if (m_mem_allocated){
		USBSerial.println("Bez - Releasing allocated memory");
		releaseMemory();
	}
	int span_size = sizeof(Span);
	USBSerial.print("Bez - Span size: ");
	USBSerial.println(span_size);
	int memory_needed = m_span_count * span_size;
	USBSerial.print("Bez - Spans: ");
	USBSerial.println(m_span_count);
	USBSerial.print("Bez - Memory needed: ");
	USBSerial.println(memory_needed);
	USBSerial.println("Bez - Allocating memory for span array");
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
		USBSerial.println("Bez - Creating ctrl pt subset");
		OrderedPair ctrl_pt_subset[4] = { m_ctrl_pts[0 + i * PT_CT], m_ctrl_pts[0 + i * PT_CT + adjust], 
			m_ctrl_pts[1 + i * PT_CT + adjust], m_ctrl_pts[1 + i * PT_CT + 2 * adjust] };
		// Don't pass a garbage address to the pointer parameter for the first span
		Span* prev_span = i == 0 ? NULL : &m_spans[i - 1];
		USBSerial.println("Bez - Creating new span");
		m_spans[i] = Span(ctrl_pt_subset, prev_span);
	}
	USBSerial.println("Bez - Setting current span pointer");
	m_cur_span = &m_spans[0];
	USBSerial.println("Bez - Span points");
	printFirstSpanPts();
	USBSerial.println("Bez - Existing span initalization");
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
		USBSerial.println("Checking span with control pts of...");
		m_spans[i].printCtrlPts();

		if (m_spans[i].containsX(p_T)){
			// Convert parameter to t of given span and get result
			USBSerial.print("minX: ");
			USBSerial.print(m_spans[i].minX());
			USBSerial.print(" rangeX: ");
			USBSerial.print(m_spans[i].rangeX());
			USBSerial.print(" p_T: ");
			USBSerial.println(p_T);
			p_T = (p_T - m_spans[i].minX()) / m_spans[i].rangeX();
			return m_spans[i].positionAtT(p_T);
		}
	}
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
	if (m_cur_span->stepsRemaining() == end_count)
		m_cur_span = m_cur_span->nextSpan();

	m_t_steps_remain--;

	return ret;
}

void CubicBezier::incrementSize(float p_h){	
	m_t_steps_remain = 1 / p_h;
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
