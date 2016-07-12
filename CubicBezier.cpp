# include "CubicBezier.h"

/*

	This library provides the ability to compute cubic bezier curves

*/

/********** OrderedPair class functions **********/

#pragma region OrderedPair Class Functions
CubicBezier::OrderedPair::OrderedPair(float p_x, float p_y){
	val(0, p_x);			
	val(1, p_y);
}

float CubicBezier::OrderedPair::val(int p_which){
	// Enforce a valid request
	p_which = p_which < 0 ? 0 : p_which > 1 ? 1 : p_which;
	return m_vals[p_which];
}

void CubicBezier::OrderedPair::val(int p_which, float p_val){
	m_vals[p_which] = p_val;
}

float  CubicBezier::OrderedPair::x(){
	return m_vals[0];
}

float CubicBezier::OrderedPair::y(){
	return m_vals[1];
}


#pragma region Arithmatic Operator Overrides
CubicBezier::OrderedPair operator * (int p_x, const CubicBezier::OrderedPair& p_op){
	return CubicBezier::OrderedPair(p_op.m_vals[0] * p_x, p_op.m_vals[1] * p_x);
}
CubicBezier::OrderedPair operator * (const CubicBezier::OrderedPair& p_op, int p_x){
	return CubicBezier::OrderedPair(p_op.m_vals[0] * p_x, p_op.m_vals[1] * p_x);
}
CubicBezier::OrderedPair operator + (int p_x, const CubicBezier::OrderedPair& p_op){
	return CubicBezier::OrderedPair(p_op.m_vals[0] + p_x, p_op.m_vals[1] * p_x);
}
CubicBezier::OrderedPair operator + (const CubicBezier::OrderedPair& p_op1, const CubicBezier::OrderedPair& p_op2){
	return CubicBezier::OrderedPair(p_op1.m_vals[0] + p_op2.m_vals[0], p_op1.m_vals[1] + p_op2.m_vals[1]);
}
CubicBezier::OrderedPair operator - (int p_x, const CubicBezier::OrderedPair& p_op){
	return CubicBezier::OrderedPair(p_op.m_vals[0] - p_x, p_op.m_vals[1] * p_x);
}
CubicBezier::OrderedPair operator - (const CubicBezier::OrderedPair& p_op){
	return CubicBezier::OrderedPair(-p_op.m_vals[0], -p_op.m_vals[1]);
}
#pragma endregion

#pragma endregion

/********** Span class functions **********/

#pragma region Span Class Functions
CubicBezier::Span::Span(OrderedPair* p_ctrl_pts, Span* p_prev_span){		
	m_ctrl_pts = p_ctrl_pts;
	m_next_span = NULL;
	m_prev_span = p_prev_span;	
	m_prev_span->nextSpan(this);
	setCoeffs();
}

void CubicBezier::Span::setCoeffs(){
	m_coeffs[0] = (-m_ctrl_pts[0]) + (3 * m_ctrl_pts[1]) + (-3 * m_ctrl_pts[2]) + m_ctrl_pts[3];
	m_coeffs[1] = (3 * m_ctrl_pts[0]) + (-6 * m_ctrl_pts[1]) + (3 * m_ctrl_pts[2]);
	m_coeffs[2] = (-3 * m_ctrl_pts[0]) + (3 * m_ctrl_pts[1]);
	m_coeffs[3] = m_ctrl_pts[0];	
}

/**
	@return The number of steps in this segment
*/
void CubicBezier::Span::incrementSizeFromX(float p_x){
	// Find h as x fraction of total span X range
	float h = p_x / rangeX();
	incrementSize(h);
	m_t_steps_remain = (int)round(1 / h);
}

void CubicBezier::Span::incrementSize(float p_h){
	m_h = p_h;
	initForwardDiff();
}

float CubicBezier::Span::incrementSize(){
	return m_h;
}

void CubicBezier::Span::initForwardDiff(){
	// Set the initial forward differences
	m_fdiff_vals[0] = m_coeffs[3];
	m_fdiff_vals[1] = m_coeffs[0] * pow(m_h, 3);
	m_fdiff_vals[2] = m_coeffs[1] * pow(m_h, 2);
	m_fdiff_vals[3] = m_coeffs[2] * m_h;

	// Zero the t step counter
	m_t = 0;
}

int CubicBezier::Span::stepsRemaining(){
	return m_t_steps_remain;
}
	
CubicBezier::OrderedPair CubicBezier::Span::positionAtNextT(){
	const int FORWARD_DIFFS = 3;
	for (int i = 0; i < FORWARD_DIFFS; i++){
		m_fdiff_vals[i] + m_fdiff_vals[i + 1];
	}
	m_t += m_h;
	m_t_steps_remain--;
	return m_fdiff_vals[0];
}

CubicBezier::OrderedPair CubicBezier::Span::positionAtT(float p_t){
	float pos[2];
	for (int i = 0; i < 2; i++){
		// B(t) = At^3 + Bt^2 + Ct + D
		pos[i] = m_coeffs[0].val(i) * pow(p_t, 3) + m_coeffs[1].val(i) * pow(p_t, 2) + m_coeffs[2].val(i) * p_t + m_coeffs[3].val(i);
	}
	return OrderedPair(pos[0], pos[1]);
}

boolean CubicBezier::Span::containsX(float p_x){
	if (m_ctrl_pts[0].x() <= p_x && p_x <= m_ctrl_pts[3].x())
		return true;
	else
		return false;
}

boolean CubicBezier::Span::containsX(float p_x){
	if (m_ctrl_pts[0].y() <= p_x && p_x <= m_ctrl_pts[3].y())
		return true;
	else
		return false;
}

float CubicBezier::Span::minX(){
	return m_ctrl_pts[0].x();
}

float CubicBezier::Span::maxX(){
	return m_ctrl_pts[g_MAX_CTRL_PTS-1].x();
}

float CubicBezier::Span::rangeX(){
	return maxX() - minX();
}

float CubicBezier::Span::minY(){
	return m_ctrl_pts[0].y();
}

float CubicBezier::Span::maxY(){
	return m_ctrl_pts[g_MAX_CTRL_PTS-1].y();
}

float CubicBezier::Span::rangeY(){
	return maxY() - minY();
}


#pragma endregion

/********** CubicBezier class functions **********/

#pragma region CubicBezier Class Functions

/*
	If specifying only knots, inner control points are set equal
	to their nearest knot, i.e.	p1 = p0 and p2 = p3, resulting
	in a linear spline. Control points p1 and p2 for each span
	may be modified later.
*/
CubicBezier::CubicBezier(OrderedPair* p_ctrl_pts, int p_knot_count, boolean p_only_knots){
	// Must have at least 2 knots to define a spline
	if (p_knot_count < 2)
		return;
	m_ctrl_pts = p_ctrl_pts;	
	m_knot_count = p_knot_count;
	m_span_count = p_knot_count - 1;
	m_mem_allocated = false;
	m_only_knots = p_only_knots;
	initializeSpans();
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
	// Dynamically allocate memory for span array
	if (m_mem_allocated)
		releaseMemory();
	m_spans = (Span *)malloc(m_span_count * sizeof(Span));
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

CubicBezier::OrderedPair CubicBezier::positionAtT(float p_t){

	float max_val;
	if (m_only_knots)
		max_val = m_ctrl_pts[m_knot_count - 1].x();
	else
		m_ctrl_pts[m_knot_count * 2 - 1].x();
	
	// Parameter t as x position
	p_t = p_t * max_val;

	// Determine which span to check
	for (int i = 0; i < m_span_count; i++){
		if (m_spans[i].containsX(p_t)){
			// Convert parameter to t of given span and get result
			p_t = (p_t - m_spans[i].minX()) / m_spans[i].rangeX();
			return m_spans[i].positionAtT(p_t);
		}
	}
}

CubicBezier::OrderedPair CubicBezier::positionAtNextT(){
	
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
	if (m_cur_span->stepsRemaining == end_count)
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
