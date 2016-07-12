# include "CubicBezier.h"

/*

	This library provides the ability to compute cubic bezier curves

*/

/********** OrderedPair class functions **********/

#pragma region OrderedPair Class Functions
CubicBezier::OrderedPair::OrderedPair(float p_a, float p_b){
	val(0, p_a);			
	val(1, p_b);
}

float CubicBezier::OrderedPair::val(int p_which){
	// Enforce a valid request
	p_which = p_which < 0 ? 0 : p_which > 1 ? 1 : p_which;
	return m_vals[p_which];
}

void CubicBezier::OrderedPair::val(int p_which, float p_val){
	m_vals[p_which] = p_val;
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
CubicBezier::Span::Span(OrderedPair* p_ctrl_pts){		
	m_ctrl_pts = p_ctrl_pts;
	setCoeffs();
}

void CubicBezier::Span::setCoeffs(){
	m_coeffs[0] = (-m_ctrl_pts[0]) + (3 * m_ctrl_pts[1]) + (-3 * m_ctrl_pts[2]) + m_ctrl_pts[3];
	m_coeffs[1] = (3 * m_ctrl_pts[0]) + (-6 * m_ctrl_pts[1]) + (3 * m_ctrl_pts[2]);
	m_coeffs[2] = (-3 * m_ctrl_pts[0]) + (3 * m_ctrl_pts[1]);
	m_coeffs[3] = m_ctrl_pts[0];	
}

void CubicBezier::Span::incrementSize(float p_h){
	m_h = p_h;
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

CubicBezier::OrderedPair CubicBezier::Span::positionAtNextT(){
	const int FORWARD_DIFFS = 3;
	for (int i = 0; i < FORWARD_DIFFS; i++){
		m_fdiff_vals[i] + m_fdiff_vals[i + 1];
	}
	m_t += m_h;
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
	m_knot_count = p_knot_count;
	m_span_count = p_knot_count - 1;
	m_mem_allocated = false;
	initializeSpans(p_ctrl_pts, p_only_knots);
}

// Default destructor
CubicBezier::~CubicBezier(){
	releaseMemory();
}

void CubicBezier::releaseMemory(){
	if (m_mem_allocated)
		free(m_spans);
}

void CubicBezier::initializeSpans(OrderedPair* p_ctrl_pts, boolean p_only_knots){
	// Dynamically allocate memory for span array
	if (m_mem_allocated)
		releaseMemory();
	m_spans = (CubicBezier::Span *)malloc(m_span_count * sizeof(CubicBezier::Span));
	m_mem_allocated = true;

	// Extract control points for each span and create new span objects
	int adjust = p_only_knots ? 0 : 1;		// Adjust the element being accessed, depending on whether only knots have been included
	int PT_CT = 4;
	for (int i = 0; i < m_span_count; i++){
		OrderedPair ctrl_pt_subset[4] = { p_ctrl_pts[0 + i * PT_CT], p_ctrl_pts[0 + i * PT_CT + adjust], 
			p_ctrl_pts[1 + i * PT_CT + adjust], p_ctrl_pts[1 + i * PT_CT + 2 * adjust] };
		m_spans[i] = CubicBezier::Span(ctrl_pt_subset);
	}
}


#pragma endregion
