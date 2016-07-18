#include "CubicBezier.h"

/*
	This library provides the ability to compute cubic Bezier curves.
	An optimized forward differencing method may be used when values
	along the curve are desired in fixed t parameter increments.
*/

/********** OrderedPair class functions **********/

#pragma region OrderedPair Class Functions

OrderedPair::OrderedPair(){}

OrderedPair::OrderedPair(float p_x, float p_y){	
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

float OrderedPair::x(){
	return m_vals[0];
}

void OrderedPair::x(int p_x){
	m_vals[0] = p_x;
}

float OrderedPair::y(){
	return m_vals[1];
}

void OrderedPair::y(int p_y){
	m_vals[1] = p_y;
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
OrderedPair operator * (double p_x, const OrderedPair& p_op){
	return OrderedPair(p_op.m_vals[0] * p_x, p_op.m_vals[1] * p_x);
}
OrderedPair operator * (const OrderedPair& p_op, double p_x){
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
	// Zero the t step counter
	m_T = 0;
}

float Span::incrementSize(){
	return m_h;
}

int Span::stepsRemaining(){
	return m_t_steps_remain;
}
	
OrderedPair Span::positionAtNextT(){		
	int this_T = m_T;
	m_T += m_h;
	m_t_steps_remain--;
	return positionAtT(this_T);
}

void Span::setCoeffs(){
	// Calculate the coefficients
	OrderedPair m_coeff_A = (-m_ctrl_pts[0]) + (3 * m_ctrl_pts[1]) + (-3 * m_ctrl_pts[2]) + m_ctrl_pts[3];
	OrderedPair m_coeff_B = (3 * m_ctrl_pts[0]) + (-6 * m_ctrl_pts[1]) + (3 * m_ctrl_pts[2]);
	OrderedPair m_coeff_C = (-3 * m_ctrl_pts[0]) + (3 * m_ctrl_pts[1]);
	OrderedPair m_coeff_D = m_ctrl_pts[0];
}

OrderedPair Span::positionAtT(float p_T){		

	float pos[2];
	for (int i = 0; i < 2; i++){
		// B(t) = At^3 + Bt^2 + Ct + D
		// Calculate via Horner's rule
		pos[i] = m_coeff_D.val(i);				
		pos[i] += p_T * m_coeff_C.val(i);		
		p_T *= p_T;
		pos[i] += p_T * m_coeff_B.val(i);
		p_T *= p_T;
		pos[i] += p_T * m_coeff_A.val(i);		
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

OrderedPair *Span::ctrlPts(){
	return m_ctrl_pts;
}

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

void CubicBezier::knotCount(int p_count){
	m_knot_count = p_count;
	m_span_count = m_knot_count - 1;
	const int PTS_PER_SPAN = 4;
	/*
		Allocate memory for control points here rather
		than pointing to an external point array
	*/
	int pt_count = m_span_count * PTS_PER_SPAN;
	m_ctrl_pts = (OrderedPair *)malloc(pt_count * sizeof(OrderedPair));
	for (int i = 0; i < pt_count; i++){
		m_ctrl_pts[i] = OrderedPair();
	}

	m_next_x = 0;
	m_next_y = 0;
}

int CubicBezier::knotCount(){
	return m_knot_count;
}

void CubicBezier::releaseMemory(){
	if (m_mem_allocated)
		free(m_spans);
}

void CubicBezier::setNextX(float p_x){
	OrderedPair op = m_ctrl_pts[m_next_x];
	op.x(p_x);
	m_ctrl_pts[m_next_x] = op;
	m_next_x++;
}

void CubicBezier::setNextY(float p_y){
	OrderedPair op = m_ctrl_pts[m_next_y];
	op.y(p_y);
	m_ctrl_pts[m_next_y] = op;
	m_next_y++;
}

void CubicBezier::initSpans(){
		
	// Dynamically allocate memory for span array
	if (m_mem_allocated){			
		releaseMemory();
	}
	size_t span_size = sizeof(Span);		
	size_t memory_needed = m_span_count * span_size;	
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

Span *CubicBezier::getSpan(int p_which){
	return &m_spans[p_which];
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

float CubicBezier::incrementSize(){
	return m_h;
}

int CubicBezier::stepsRemaining(){
	return m_t_steps_remain;
}

#pragma endregion
