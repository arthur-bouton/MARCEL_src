

class elapsedMicros
{
	private:
		unsigned long us;
	public:
		elapsedMicros(void) { us = micros(); }
		elapsedMicros(unsigned long val) { us = micros() - val; }
		elapsedMicros(const elapsedMicros &orig) { us = orig.us; }
		operator unsigned long () const { return micros() - us; }
		elapsedMicros & operator = (const elapsedMicros &rhs) { us = rhs.us; return *this; }
		elapsedMicros & operator = (unsigned long val) { us = micros() - val; return *this; }
		elapsedMicros & operator -= (unsigned long val)      { us += val ; return *this; }
		elapsedMicros & operator += (unsigned long val)      { us -= val ; return *this; }
		elapsedMicros operator - (int val) const           { elapsedMicros r(*this); r.us += val; return r; }
		elapsedMicros operator - (unsigned int val) const  { elapsedMicros r(*this); r.us += val; return r; }
		elapsedMicros operator - (long val) const          { elapsedMicros r(*this); r.us += val; return r; }
		elapsedMicros operator - (unsigned long val) const { elapsedMicros r(*this); r.us += val; return r; }
		elapsedMicros operator + (int val) const           { elapsedMicros r(*this); r.us -= val; return r; }
		elapsedMicros operator + (unsigned int val) const  { elapsedMicros r(*this); r.us -= val; return r; }
		elapsedMicros operator + (long val) const          { elapsedMicros r(*this); r.us -= val; return r; }
		elapsedMicros operator + (unsigned long val) const { elapsedMicros r(*this); r.us -= val; return r; }
};
