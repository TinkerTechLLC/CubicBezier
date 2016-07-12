// Bezier.h

/*
	Forward differencing reference:
	http://www.drdobbs.com/forward-difference-calculation-of-bezier/184403417
*/

#ifndef _BEZIER_h
#define _BEZIER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class CubicBezier
{
	public:
		/*
			If specifying only knots, inner control points are set equal
			to their nearest knot, i.e.	p1 = p0 and p2 = p3, resulting
			in a linear spline. Control points p1 and p2 for each span
			may be modified later.
		*/
		
		class OrderedPair {
		public:
			OrderedPair();
			OrderedPair(float p_x, float p_y);
			float val(int p_which);
			void val(int p_which, float p_val);						

			friend CubicBezier::OrderedPair operator *(int val, const CubicBezier::OrderedPair &op);
			friend CubicBezier::OrderedPair operator *(const CubicBezier::OrderedPair &op, int val);
			friend CubicBezier::OrderedPair operator +(int val, const CubicBezier::OrderedPair &op);
			friend CubicBezier::OrderedPair operator + (const CubicBezier::OrderedPair& p_op1, const CubicBezier::OrderedPair& p_op2);
			friend CubicBezier::OrderedPair operator -(int val, const CubicBezier::OrderedPair &op);
			friend CubicBezier::OrderedPair operator -(const CubicBezier::OrderedPair &op);
			

		private:
			float* m_vals;
		};

		class Span {
		public:
			Span(OrderedPair* p_ctrl_pts);			
			OrderedPair positionAtT(float p_t);
			OrderedPair positionAtNextT();
			void incrementSize(float p_h);
			float incrementSize();
			void initForwardDiff();

		private:			
			void setCoeffs();
			OrderedPair m_coeffs[4];
			OrderedPair* m_ctrl_pts;
			float* m_abscissa;
			float* m_position;
			float m_h;				// The t step size for forward differencing calculation
			float m_t;
			OrderedPair m_fdiff_vals[4];
			
		};		
		
		int m_knot_count;
		Span* m_spans;
		int m_span_count;
		CubicBezier(OrderedPair* p_ctrl_pts, int p_knot_count, boolean p_only_knots);
		~CubicBezier();

	private:
		boolean m_mem_allocated;
		void initializeSpans(OrderedPair* p_ctrl_pts, boolean p_only_knots);
		void releaseMemory();

};




#endif
