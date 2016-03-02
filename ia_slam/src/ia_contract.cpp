class Detection : public Ctc {
public:
	/*
	 * The contractor is for a specific beacon "b" which
	 * is specified in argument of the constructor.
	 */
	Detection(int b) : Ctc(2), b(b) {
		Variable x(2);
		// This function will be created once for the T time steps!
		dist = new Function(x,sqrt(sqr(x[0]-beacons[b][0])+sqr(x[1]-beacons[b][1])));
	}

	/*
	 * Allow to set the time dynamically
	 */
	void set_time(int t) {
		this->t=t;
	}

	void contract(IntervalVector& x) {
		// by simplicity, we call the backward
		// operator on the function directly
		dist->backward(d[t][b],x);
	}


protected:
	int b;          // beacon number
	int t;          // time number
	Function* dist; // distance function
};
//![detection]

//![speed]
/*
 * Contractor for the "speed" constraint.
 *
 * This is a contractor parametrized by the time "t".
 * It means that a call to contract() must be
 * preceded by a call to set_time(t).
 */
class Speed : public Ctc {
public:
	Speed() : Ctc(2) {
		Variable a(2);
		Variable b(2);
		delta = new Function(a,b,b-a);
	}

	void contract(IntervalVector& x) {
		delta->backward(v[t],x);
	}

	void set_time(int t) {
		this->t=t;
	}

protected:
	int t;
	Function* delta;
};
//![speed]


//![scan]
/*
 * Scanning contractor that aggregates
 * the N detections occurring at a given time t.
 */
class Scan : public Ctc {
public:
	Scan() : Ctc(2) {

		// The N detections
		detect = new Detection*[N];

		// The q-intersection is created as before,
		// using a temporary vector "cdist"
		vector<Ctc*> cdist;
		for (int b=0; b<N; b++) {
			cdist.push_back(detect[b]=new Detection(b));
		}
		qinter = new CtcQInter(cdist,N-NB_OUTLIERS);
	}

	void contract(IntervalVector& x) {
		qinter->contract(x);
	}

	void set_time(int t) {
		// we set the time of each sub-contractor
		for (int i=0; i<N; i++)
			detect[i]->set_time(t);
	}

protected:
	Detection** detect;
	CtcQInter* qinter;
	int t;
};
//![scan]
