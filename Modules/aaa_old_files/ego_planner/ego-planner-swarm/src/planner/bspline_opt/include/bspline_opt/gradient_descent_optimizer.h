#ifndef _GRADIENT_DESCENT_OPT_H_
#define _GRADIENT_DESCENT_OPT_H_

#include <iostream>
#include <vector>
#include <Eigen/Eigen>

using namespace std;

class GradientDescentOptimizer
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef double (*objfunDef)(const Eigen::VectorXd &x, Eigen::VectorXd &grad, bool &force_return, void *data);
  enum RESULT
  {
    FIND_MIN,
    FAILED,
    RETURN_BY_ORDER,
    REACH_MAX_ITERATION
  };

  GradientDescentOptimizer(int v_num, objfunDef objf, void *f_data)
  {
    variable_num_ = v_num;
    objfun_ = objf;
    f_data_ = f_data;
  };

  void set_maxiter(int limit) { iter_limit_ = limit; }
  void set_maxeval(int limit) { invoke_limit_ = limit; }
  void set_xtol_rel(double xtol_rel) { xtol_rel_ = xtol_rel; }
  void set_xtol_abs(double xtol_abs) { xtol_abs_ = xtol_abs; }
  void set_min_grad(double min_grad) { min_grad_ = min_grad; }

  RESULT optimize(Eigen::VectorXd &x_init_optimal, double &opt_f);

private:
  int variable_num_{0};
  int iter_limit_{1e10};
  int invoke_limit_{1e10};
  double xtol_rel_;
  double xtol_abs_;
  double min_grad_;
  double time_limit_;
  void *f_data_;
  objfunDef objfun_;
};

#endif
