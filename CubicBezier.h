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

class OrderedPair {

		public:
			OrderedPair();
			OrderedPair(float p_a, float p_b);
			//~OrderedPair();
			float val(int p_which);
			void val(int p_which, float p_val);		
			float x();
			float y();			

			friend OrderedPair operator *(int val, const OrderedPair &op);
			friend OrderedPair operator *(const OrderedPair &op, int val);
			friend OrderedPair operator *(float val, const OrderedPair &op);
			friend OrderedPair operator *(const OrderedPair &op, float val);
			friend OrderedPair operator *(double val, const OrderedPair &op);
			friend OrderedPair operator *(const OrderedPair &op, double val);
			friend OrderedPair operator +(int val, const OrderedPair &op);
			friend OrderedPair operator +(const OrderedPair &op, int val);
			friend OrderedPair operator +(float val, const OrderedPair &op);
			friend OrderedPair operator +(const OrderedPair &op, float val);
			friend OrderedPair operator + (const OrderedPair& p_op1, const OrderedPair& p_op2);
			friend OrderedPair operator -(int val, const OrderedPair &op);
			friend OrderedPair operator -(const OrderedPair &op);

		private:
			float m_vals[2];	// The (a, b) values
};

class Span {

	public:
		Span();
		Span(OrderedPair *p_ctrl_pts, Span* p_prev_span);						
		OrderedPair positionAtT(float p_t);
		OrderedPair positionAtNextT();
		void incrementSize(float p_h);
		void incrementSizeFromX(float p_h);
		float incrementSize();			
		bool containsX(float p_x);
		bool containsY(float p_y);
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
		
		OrderedPair *ctrlPts();
		int id();

	private:			
		void setCoeffs();
		OrderedPair m_coeff_A;
		OrderedPair m_coeff_B;
		OrderedPair m_coeff_C;
		OrderedPair m_coeff_D;
		Span *m_next_span;
		Span *m_prev_span;
		static const int g_MAX_CTRL_PTS = 4;			
		OrderedPair m_ctrl_pts[g_MAX_CTRL_PTS];
		float m_h;				// The t step size for forward differencing calculation
		float m_T;
		int m_t_steps_remain;
		static int g_id_gen;
		int m_id;	
};	

class CubicBezier
{
	public:

		/********** Public Variables **********/

		int m_knot_count;
		Span *m_spans;
		int m_span_count;
		
		/********** Public Functions **********/

		CubicBezier();
		CubicBezier(OrderedPair *p_ctrl_pts, int p_knot_count, bool p_only_knots);
		~CubicBezier();
		OrderedPair positionAtT(float p_T);
		OrderedPair positionAtNextT();
		void initForwardDiff(float p_h);
		float incrementSize();
		int stepsRemaining();
		Span *getSpan(int p_which);
		void initSpans();

	private:

		/********** Private Variables **********/

		bool m_mem_allocated;
		OrderedPair *m_ctrl_pts;
		bool m_only_knots;
		Span* m_cur_span;
		float m_h;
		float m_T;
		int m_t_steps_remain;

		/********** Private Functions **********/		
		
		void releaseMemory();
};




#endif
