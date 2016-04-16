#define _USE_MATH_DEFINES

#include <vector>
#include <string>
#include <iostream>
#include <math.h>
#include <Windows.h>

using namespace std;

#ifndef _HMM_H
#define _HMM_H
typedef vector<int> Sequence;

struct HMMResult
{
	int index;        // the index of the matched template
	double partition; // the partition of possibility in all templates

	public:
	HMMResult() : index(-1), partition(-1.0) {}

	friend bool operator>(HMMResult a, HMMResult b);
	friend bool operator>=(HMMResult a, HMMResult b);
	friend bool operator<(HMMResult a, HMMResult b);
	friend bool operator<=(HMMResult a, HMMResult b);
	friend bool operator==(HMMResult a, HMMResult b);
};

bool operator>(HMMResult a, HMMResult b)
{
	if (a.partition > b.partition) return true;
	if (a.index < b.index) return true;
	return false;
}
bool operator>=(HMMResult a, HMMResult b)
{
	if (a.partition >= b.partition) return true;
	if (a.index <= b.index) return true;
	return false;
}
bool operator<(HMMResult a, HMMResult b)
{
	if (a.partition < b.partition) return true;
	if (a.index > b.index) return true;
	return false;
}
bool operator<=(HMMResult a, HMMResult b)
{
	if (a.partition <= b.partition) return true;
	if (a.index >= b.index) return true;
	return false;
}
bool operator==(HMMResult a, HMMResult b)
{
	if (a.partition == b.partition && a.index == b.partition) return true;
	return false;
}

class HMM
{
#if defined _HMM_DEBUG
	public:
#else
	private:
#endif
		int K;					// number of possible emissions 
		int N;					// number of possible states
		int T;          // number of points
		double *A;			// transision matrix
		double *B;			// emission matrix
		double *Pi;			// initial distribution
		double thres;		// the threshold used to match
		vector<Sequence> samples;
										// initial sequences needed to generate the model
		Sequence C;			// count for each corresponding sequence

	public:
		// constructor & destructor
	  HMM();
		HMM(int _K, int _N, int _T, double _thres = 0.5);
		HMM(HMM &_hmm);
		~HMM();

		// initialize the model
		bool setA(double *_A);
		bool setB(double *_B);
		bool setPi(double *_Pi);
		void outA();
		void outB();
		void outPi();
		void outSamples();
		bool pushSample(Sequence sample, int _C = 1);
		Sequence& getSample(int index);
		bool generateHMM(double thres);

		// match new samples
		HMMResult match(Sequence sample);

		// operators
		void operator=(HMM &_hmm);

		void clear()
		{
			*this = HMM();
		}
};

HMM::HMM()
{
	K = 0;
	N = 0;
	T = 0;
	A = NULL;
	B = NULL;
	Pi = NULL;
}

HMM::HMM(int _K, int _N, int _T, double _thres) :
K(_K), N(_N), T(_T), thres(_thres)
{
#if defined _HMM_DEBUG
	cout << "Constructing HMM class" << endl;
#endif
	A = new double[_N*_N];
	for (int i = 0; i < _N; i++)
	{
		for (int j = 0; j < _N; j++)
		{
			A[i*_N + j] = 1.0 / _N;
		}
	}
#if defined _HMM_DEBUG
	outA();
#endif
	B = new double[_N*_K];
	for (int i = 0; i < _N; i++)
	{
		for (int j = 0; j < _K; j++)
		{
			B[i*_K + j] = 1.0 / _K;
		}
	}
#if defined _HMM_DEBUG
	outB();
#endif
	Pi = new double[_N];
	for (int i = 0; i < _N; i++)
	{
		Pi[i] = 1.0 / _N;
	}
#if defined _HMM_DEBUG
	outPi();
#endif
}

HMM::HMM(HMM &_hmm)
{
	*this = _hmm;
}

HMM::~HMM()
{
	if (A) delete[] A;
	if (B) delete[] B;
	if (Pi) delete[] Pi;
	samples.clear();
	C.clear();
}

bool HMM::setA(double *_A)
{
	if (!_A)
	{
		return false;
	}
	else
	{
		if (A) delete[] A;
		A = new double[N*N];
		for (int i = 0; i < N; i++)
		{
			for (int j = 0; j < N; j++)
			{
				A[i*N + j] = _A[i*N + j];
			}
		}
	}
#if defined _HMM_DEBUG
	outA();
#endif
	return true;
}

bool HMM::setB(double *_B)
{
	if (!_B)
	{
		return false;
	}
	else
	{
		if (B) delete[] B;
		B = new double[N*K];
		for (int i = 0; i < N; i++)
		{
			for (int j = 0; j < K; j++)
			{
				B[i*K + j] = _B[i*K + j];
			}
		}
	}
#if defined _HMM_DEBUG
	outB();
#endif
	return true;
}

bool HMM::setPi(double *_Pi)
{
	if (!_Pi)
	{
		return false;
	}
	else
	{
		if (Pi) delete[] Pi;
		Pi = new double[N];
		for (int i = 0; i < N; i++)
		{
			Pi[i] = _Pi[i];
		}
	}
#if defined _HMM_DEBUG
	outPi();
#endif
	return true;
}

void HMM::outA()
{
	if (!A)
	{
		return;
	}
	else
	{
		cout << "A:" << endl;
		for (int i = 0; i < N; i++)
		{
			cout << "A[" << i << "]: ";
			for (int j = 0; j < N; j++)
			{
				cout << A[i*N + j] << " ";
			}
			cout << endl;
		}
	}
}

void HMM::outB()
{
	if (!B)
	{
		return;
	}
	else
	{
		cout << "B:" << endl;
		for (int i = 0; i < N; i++)
		{
			cout << "B[" << i << "]: ";
			for (int j = 0; j < K; j++)
			{
				cout << B[i*K + j] << " ";
			}
			cout << endl;
		}
	}
}

void HMM::outPi()
{
	if (!A)
	{
		return;
	}
	else
	{
		cout << "Pi:";
		for (int i = 0; i < N; i++)
		{
			cout << Pi[i] << " ";
		}
		cout << endl;
	}
}

void HMM::outSamples()
{
	int i = 0;
	for (vector<Sequence>::iterator p = samples.begin(); p != samples.end(); p++, i++)
	{
		cout << "sample #" << i << " C=" << C[i] << endl;
		for (int j = 0; j < T; j++)
		{
			cout << samples[i][j] << " ";
		}
		cout << endl;
	}
}

bool HMM::pushSample(Sequence sample, int _C)
{
	samples.push_back(sample);
	C.push_back(_C);
	return true;
}

Sequence& HMM::getSample(int index)
{
	if (index < samples.size() && index >= 0)
	{
		return samples[index];
	}
	else
	{
		return Sequence();
	}
}

bool HMM::generateHMM(double thres)
{
	double likelihood = 0;
	double plikelihood = -99999999999;
  int count = 0;
	int size = samples.size();
	while (true)
	{
#if defined _HMM_DEBUG
    cout << endl << endl;
    cout << "Round #" << count << endl;
    count++;
    cout << "ln(pastLikelihood) = " << plikelihood << endl;
#endif
		// ¦Á[i][t]: the probability of seeing y1 through yt and xt being in state i.
		double *alpha = new double[N*T*size];
		// ¦Â[i][t]: the probability of seeing yt+1 through yT and xt being in state i.
		double *beta = new double[N*T*size];
		// 
		double *gama = new double[N*N*T*size];
		// 
		double *epsilon = new double[N*T*size];
		// likelihood of each sample
		double *like = new double[size];
		// initialize
		ZeroMemory(alpha, sizeof(double)*N*T*size);
		ZeroMemory(beta, sizeof(double)*N*T*size);
		ZeroMemory(gama, sizeof(double)*N*N*T*size);
		ZeroMemory(epsilon, sizeof(double)*N*T*size);
		ZeroMemory(like, sizeof(double)*size);

		// calculation of ¦Á
		for (int index = 0; index < size; index++)
		{
			Sequence& seq = samples[index];
			for (int t = 0; t < T; t++)
			{
				if (t == 0)
				{
					for (int state = 0; state < N; state++)
					{
						alpha[index*N*T + state*T] = Pi[state] * B[state*K + seq[0]];
					}
				}
				else
				{
					for (int state = 0; state < N; state++)
					{
						double sum = 0;
						for (int lstate = 0; lstate < N; lstate++)
						{
							sum += alpha[index*N*T + lstate*T + t - 1] * A[lstate*N + state];
						}
						alpha[index*N*T + state*T + t] = B[state*K + seq[t]] * sum;
						if (t == T - 1)
						{
							like[index] += alpha[index*N*T + state*T + t];
						}
					}
				}
			}
		}
#if defined _HMM_DEBUG
		cout << "Alpha: " << endl;
		for (int index = 0; index < size; index++)
		{
			cout << "Sample #" << index << endl;
			for (int i = 0; i < N; i++)
			{
				cout << i << ": ";
				for (int t = 0; t < T; t++)
				{
					cout << alpha[index*N*T + i*T + t] << " ";
				}
				cout << endl;
			}
		}
#endif

		// calculation of ¦Â
		for (int index = 0; index < size; index++)
		{
			Sequence& seq = samples[index];
			for (int state = 0; state < N; state++)
			{
				beta[index*N*T + state*T + T - 1] = 1;
			}
			for (int t = T - 2; t >= 0; t--)
			{
				for (int state = 0; state < N; state++)
				{
					beta[index*N*T + state*T + t] = 0;
					for (int nstate = 0; nstate < N; nstate++)
					{
						beta[index*N*T + state*T + t] +=
							beta[index*N*T + nstate*T + t + 1] *
							A[state*N + nstate] * B[nstate*K + seq[t + 1]];
					}
				}
			}
		}
#if defined _HMM_DEBUG
		cout << "Beta: " << endl;
		for (int index = 0; index < size; index++)
		{
			cout << "Sample #" << index << endl;
			for (int i = 0; i < N; i++)
			{
				cout << i << ": ";
				for (int t = 0; t < T; t++)
				{
					cout << beta[index*N*T + i*T + t] << " ";
				}
				cout << endl;
			}
		}
#endif

		// calculation of ¦Ã
		for (int index = 0; index < size; index++)
		{
			Sequence& seq = samples[index];
			double prh = 0;
			for (int state = 0; state < N; state++)
			{
				prh += alpha[index*N*T + state*T + T - 1];
			}
			for (int t = 0; t < T - 1; t++)
			{
				for (int state = 0; state < N; state++)
				{
					for (int nstate = 0; nstate < N; nstate++)
					{
            gama[index*T*N*N + t*N*N + state*N + nstate] =
              alpha[index*T*N + state*T + t] * A[state*N + nstate] *
              beta[index*T*N + nstate*T + t + 1] * B[nstate*K + seq[t + 1]] / prh;
					}
				}
			}
		}
#if defined _HMM_DEBUG
		cout << "Gama: " << endl;
		for (int index = 0; index < size; index++)
		{
			cout << "Sample #" << index << endl;
      for (int t = 0; t < T - 1; t++)
			{
        for (int i = 0; i < N; i++)
				{
          for (int j = 0; j < N; j++)
					{
            cout << "Gama[" << t << "," << i << "," << j << "]: " << gama[index*N*N*T + t*N*N + i*N + j] << endl;
					}
				}
			}
		}
#endif

		// calculation of epsilon
		for (int index = 0; index < size; index++)
		{
			Sequence& seq = samples[index];
			for (int t = 0; t < T; t++)
			{
				double temp = 0;
				for (int state = 0; state < N; state++)
				{
					temp += alpha[index*N*T + state*T + t] * beta[index*N*T + state*T + t];
				}
				for (int state = 0; state < N; state++)
				{
					epsilon[index*N*T + state*T + t] = (alpha[index*N*T + state*T + t] * beta[index*N*T + state*T + t]) / temp;
				}
			}
		}
#if defined _HMM_DEBUG
		cout << "Epsilon: " << endl;
		for (int index = 0; index < size; index++)
		{
			cout << "Sample #" << index << endl;
			for (int i = 0; i < N; i++)
			{
				cout << i << ": ";
				for (int t = 0; t < T; t++)
				{
					cout << epsilon[index*N*T + i*T + t] << " ";
				}
				cout << endl;
			}
		}
#endif

		// update
#ifdef _HMM_DEBUG
		cout << "Update process" << endl;
#endif
		// Pi
#ifdef _HMM_DEBUG
		cout << "Calculation of Pi" << endl;
#endif
    double sum = 0;
    for (int state = 0; state < N; state++)
    {
      Pi[state] = 0;
      for (int index = 0; index < size; index++)
      {
        Pi[state] += epsilon[index*N*T + state*T] * C[index];
      }
    }
    for (int state = 0; state < N; state++)
    {
      sum += Pi[state];
    }
    for (int state = 0; state < N; state++)
    {
      Pi[state] /= sum;
#ifdef _HMM_DEBUG
      cout << "Pi[" << state << "] = " << Pi[state] << endl;
#endif
    }
		
		// A
#ifdef _HMM_DEBUG
		cout << "Calculation of A" << endl;
#endif
    ZeroMemory(A, sizeof(double)*N*N);
    for (int i = 0; i < N; i++)
    {
      double sum = 0;
      for (int j = 0; j < N; j++)
      {
        for (int index = 0; index < size; index++)
        {
          for (int t = 0; t < T - 1; t++)
          {
            A[i*N + j] += gama[index*N*N*T + t*N*N + i*N + j] *C[index];
          }
        }
        sum += A[i*N + j];
      }
      for (int j = 0; j < N; j++)
      {
        A[i*N + j] /= sum;
#ifdef _HMM_DEBUG
        cout << "A[" << i << "][" << j << "] = " << A[i*N + j] << endl;
#endif
      }
    }
    

		// B
#ifdef _HMM_DEBUG
		cout << "Calculation of B" << endl;
#endif
    ZeroMemory(B, sizeof(double)*N*K);
    for (int i = 0; i < N; i++)
    {
      double sum = 0;
      for (int j = 0; j < K; j++)
      {
        for (int index = 0; index < size; index++)
        {
          Sequence& seq = samples[index];
          for (int t = 0; t < T; t++)
          {
            if (seq[t] == j)
            {
              B[i*K + j] += epsilon[index*N*T + i*T + t] * C[index];
            }
          }
        }
        sum += B[i*K + j];
      }
      for (int j = 0; j < K; j++)
      {
        B[i*K + j] /= sum;
#ifdef _HMM_DEBUG
        cout << "B[" << i << "][" << j << "] = " << B[i*K + j] << endl;
#endif
      }
    }

		// calculate the likelihood using log
    likelihood = 0;
		for (int index = 0; index < size; index++)
		{
			likelihood += C[index] * log(like[index]) / log(M_E);
		}
#ifdef _HMM_DEBUG
    cout << "ln(Likelihood) = " << likelihood << endl;
    cout << "ln(pastLikelihood) = " << plikelihood << endl;
    cout << "Likelihood difference = " << fabs(likelihood - plikelihood) << endl;
    system("pause");
#endif
		delete[] alpha;
		delete[] beta;
		delete[] gama;
		delete[] epsilon;
		delete[] like;
		if (fabs(likelihood - plikelihood) <= thres || likelihood < plikelihood)
		{
			return true;
		}
    plikelihood = likelihood;
	}
}

HMMResult HMM::match(Sequence sample)
{
	int size = samples.size();
	HMMResult result;
	result.index = -1;
	result.partition = 0.0;
	double sum = 0;
	for (int index = 0; index < size; index++)
	{
		double pos = 1.0;
		Sequence& seq = samples[index];
		for (int i = 0; i < T; i++)
		{
			pos *= B[seq[i] * K + sample[i]];
		}
		sum += pos;
		if (pos>result.partition)
		{
			result.index = index;
			result.partition = pos;
		}
	}
	result.partition /= sum;
	return result;
}

void HMM::operator=(HMM &_hmm)
{
	K = _hmm.K;
	N = _hmm.N;
	thres = _hmm.thres;
	if (A) delete[] A;
	if (B) delete[] B;
	if (Pi) delete[] Pi;
	A = new double[N*N];
	B = new double[N*K];
	Pi = new double[N];
	setA(_hmm.A);
	setB(_hmm.B);
	setPi(_hmm.Pi);
	samples.clear();
	for (vector<Sequence>::iterator i = _hmm.samples.begin();
			 i != _hmm.samples.end(); i++)
	{
		samples.push_back(*i);
	}
	C.clear();
	for (Sequence::iterator i = _hmm.C.begin();
			 i != _hmm.C.end(); i++)
	{
		C.push_back(*i);
	}
}
#endif