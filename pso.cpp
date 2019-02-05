#include <iostream>
#include <random>
#include <limits>

using namespace std;

//Objective function
double square(double x)
{
	return ((x*x));
}

//Class for individual particles
class Particle
{
	private:
		double (*objective)(double pos);
		double position;
		double velocity;
		double error;
		double position_best;
		double err_best;

	public:
		Particle()
		{
			position_best = -1;
			err_best = std::numeric_limits<double>::max();
		}

		void Evaluate()
		{
			error = objective(position);
			if (error < err_best)
			{
				position_best = position;
				err_best = error;
			}
		}

		void updatePos()
		{
			position = position + velocity;
			this->Evaluate();
		}

		void updateVel(double s_param, double c_param, double w, double g_best_pos)
		{
			uniform_real_distribution<double> unif(0, 1);
                        default_random_engine re;
                        double c_rand;
                        double s_rand;
			c_rand = unif(re);
			s_rand = unif(re);
			velocity = (w*velocity) + (c_param*c_rand*(position_best - position)) + (s_param *s_rand* (g_best_pos - position));
		}

		void setPos(double pos)	{position = pos;}
		void setVel(double vel) {velocity = vel;}
		void setObjective(double (*f)(double pos)) {objective = f;}
		double getPos() {return position;}
                double getVel() {return velocity;}
                double getErr() {return error;}
};

//Class for the whole swarm
class Swarm
{	
	public:
		Swarm(double (*f)(double pos),
			int n,
			double s_const,
			double c_const,
			double w,
			double m_i,
			double p_ul,
                        double p_ll,
                        double v_ul,
                        double v_ll)
		{	
			objective = f;
			n_particles = n;
			pos_upper_lim = p_ul;
			pos_lower_lim = p_ll;
			vel_upper_lim = v_ul;
			vel_lower_lim = v_ll;
			s_param = s_const;
			c_param = c_const;
			inertia = w;
			maxIter = m_i;

			err_best = std::numeric_limits<double>::max();

			uniform_real_distribution<double> unif(0, 1);
			default_random_engine re;
			double rand_pos;
			double rand_vel;
			p = new Particle[n_particles];
			
			//Initialize particles
			for (int i = 0; i< n_particles ; i++)
			{
				rand_pos = pos_lower_lim + (pos_upper_lim - pos_lower_lim)*unif(re);
	                        rand_vel = vel_lower_lim + (vel_upper_lim - vel_lower_lim)*unif(re);
				p[i].setPos(rand_pos);
				p[i].setVel(rand_vel);
				p[i].setObjective(f);
				p[i].Evaluate();
			}

			//Find initial best errors
			for (int i = 0; i < n_particles; i++)
			{
				if (p[i].getErr() < err_best)
				{
					err_best = p[i].getErr();
					pos_best = p[i].getPos();
				}
			}
		}

		void optimize()
		{
			for (int it = 0; it < maxIter; it++)
			{	
				for (int i = 0; i < n_particles; i++)
                                {
					p[i].updateVel(s_param, c_param, inertia, pos_best);
					p[i].updatePos();
                                }

				for (int i = 0; i < n_particles; i++)
        	                {
	                                if (p[i].getErr() < err_best)
                                	{
                        	                err_best = p[i].getErr();
                	                        pos_best = p[i].getPos();
        	                        }
	                        }
			}
		}
		void showAllPos()
                {
                        for (int i = 0; i < n_particles; i++)
                        {
                                 cout<< p[i].getPos()<< '\n';
                        }
                }

                void showAllVel()
                {
                        for (int i = 0; i < n_particles; i++)
                        {
                                 cout<< p[i].getVel()<< '\n';
                        }
                }

		void showAllErrs()
		 {
                        for (int i = 0; i < n_particles; i++)
                        {
                                 cout<< p[i].getErr()<< '\n';
                        }
                }

		double getBestErr() {return err_best;}
		double getBestPos() {return pos_best;}

	private:
		double (*objective)(double pos);
		int n_particles;
		double pos_upper_lim;
		double pos_lower_lim;
		double pos_best;
		double vel_upper_lim;
		double vel_lower_lim;
		double err_best;
		double s_param;
		double c_param;
		double inertia;
		double maxIter;
		Particle * p;

};

int main()
{	
	int N = 1000;
	//cout<<"\nEnter the number of particles:\n";
	//cin>>N;

	double p_ul = 10.0;
	double p_ll = -10.0;
	double v_ul = 1.0;
	double v_ll = -1.0;

	double s_const = 1.0;
	double c_const = 1.0;
	double w = 0.5;
	double m_i = 1000;

	Swarm mySwarm(square, N, s_const, c_const, w, m_i, p_ul, p_ll, v_ul, v_ll);

	mySwarm.showAllPos();
	cout<<"\n\n";
	mySwarm.showAllVel();
	cout<<"\n\n";
	mySwarm.showAllErrs();
        cout<<"\n\n";

	mySwarm.optimize();
	double b_err = mySwarm.getBestErr();
	double b_pos = mySwarm.getBestPos();
	cout << "Best Error:\t" << b_err << "\tBest Position:\t"<< b_pos <<'\n';
	return 0;
}
