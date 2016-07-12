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
		
		/********** Public Nested Classes **********/

		class OrderedPair {
		public:
			OrderedPair();
			OrderedPair(float p_a, float p_b);
			float val(int p_which);
			void val(int p_which, float p_val);		
			float x();
			float y();			

			friend CubicBezier::OrderedPair operator *(int val, const CubicBezier::OrderedPair &op);
			friend CubicBezier::OrderedPair operator *(const CubicBezier::OrderedPair &op, int val);
			friend CubicBezier::OrderedPair operator +(int val, const CubicBezier::OrderedPair &op);
			friend CubicBezier::OrderedPair operator + (const CubicBezier::OrderedPair& p_op1, const CubicBezier::OrderedPair& p_op2);
			friend CubicBezier::OrderedPair operator -(int val, const CubicBezier::OrderedPair &op);
			friend CubicBezier::OrderedPair operator -(const CubicBezier::OrderedPair &op);

		private:
			float* m_vals;	// The (a, b) values
		};

		class Span {
		public:
			Span(OrderedPair* p_ctrl_pts, Span* p_prev_span);						
			OrderedPair positionAtT(float p_t);
			OrderedPair positionAtNextT();
			void incrementSize(float p_h);
			void incrementSizeFromX(float p_h);
			float incrementSize();			
			boolean containsX(float p_x);
			boolean containsy(float p_y);			
			float minX();
			float maxX();
			float rangeX();
			float minY();
			float maxY();
			float rangeY();
			int stepsRemaining();

			void prevSpan(Span* p_span);
			Span* prevSpan();
			void nextSpan(Span* p_span);
			Span* nextSpan();

		private:			
			void initForwardDiff();
			void setCoeffs();

			Span* m_next_span;
			Span* m_prev_span;
			static const int g_MAX_CTRL_PTS = 4;			
			OrderedPair m_coeffs[g_MAX_CTRL_PTS];
			OrderedPair* m_ctrl_pts;
			float* m_abscissa;
			float* m_position;
			float m_h;				// The t step size for forward differencing calculation
			float m_t;
			int m_t_steps_remain;
			OrderedPair m_fdiff_vals[g_MAX_CTRL_PTS];
			
			
		};		
		
		/********** Public Variables **********/

		int m_knot_count;
		Span* m_spans;
		int m_span_count;
		
		/********** Public Functions **********/

		CubicBezier(OrderedPair* p_ctrl_pts, int p_knot_count, boolean p_only_knots);
		~CubicBezier();
		OrderedPair positionAtT(float p_t);
		OrderedPair positionAtNextT();
		void incrementSize(float p_h);
		float incrementSize();
		int stepsRemaining();

	private:

		/********** Private Variables **********/

		boolean m_mem_allocated;
		OrderedPair* m_ctrl_pts;
		boolean m_only_knots;
		void initializeSpans();
		Span* m_cur_span;
		float m_h;
		float m_t;
		int m_t_steps_remain;

		/********** Private Functions **********/
		void releaseMemory();
};




#endif
