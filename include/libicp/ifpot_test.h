#pragma once

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

namespace ifopt {
  
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class TransformVariables : public VariableSet {
public:
  
  // variable set:
  // r,p,y,x,y,z
  TransformVariables() : TransformVariables("var_set_tf"){};
  TransformVariables( const std::string& name )
    : VariableSet( 6, name ),
    alpha(0.), beta(0.), gama(0.),
    x(0.), y(0.), z(0.)
  {}
    
  void SetVariables(const Eigen::VectorXd& vars) override
  {
    alpha = vars(0);
    beta = vars(1);
    gama = vars(2);
    
    x = vars(3);
    y = vars(4);
    z = vars(5);
  };
  
  VectorXd GetValues() const override
  {
    Vector6d values;
    values << alpha, beta, gama, x, y, z;
    return values;
  }
  
  VecBound GetBounds() const override
  {
    VecBound bounds(GetRows());
    for( int i = 0; i < 6; ++i )
      bounds.at(i) = NoBound;
    return bounds;
  }

private:
  double alpha, beta, gama;
  double x,y,z;
};

class TransformConstraint : public ConstraintSet {
public:
  TransformConstraint() : TransformConstraint("constraint_tf") {}

  // This constraint set just contains 1 constraint, however generally
  // each set can contain multiple related constraints.
  TransformConstraint(const std::string& name) : ConstraintSet(1, name) {}

  // The constraint value minus the constant value "1", moved to bounds.
  VectorXd GetValues() const override
  {
    VectorXd g(GetRows());
    Vector6d x = GetVariables()->GetComponent("var_set_tf")->GetValues();
    g(0) = std::pow(x(3),2) + std::pow(x(4),2) + std::pow(x(5),2);
    return g;
  };

  // The only constraint in this set is an equality constraint to 1.
  // Constant values should always be put into GetBounds(), not GetValues().
  // For inequality constraints (<,>), use Bounds(x, inf) or Bounds(-inf, x).
  VecBound GetBounds() const override
  {
    VecBound b(GetRows());
    b.at(0) = Bounds(0, 9);
    return b;
  }

  // This function provides the first derivative of the constraints.
  // In case this is too difficult to write, you can also tell the solvers to
  // approximate the derivatives by finite differences and not overwrite this
  // function, e.g. in ipopt.cc::use_jacobian_approximation_ = true
//   void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override
//   {
//     // must fill only that submatrix of the overall Jacobian that relates
//     // to this constraint and "var_set1". even if more constraints or variables
//     // classes are added, this submatrix will always start at row 0 and column 0,
//     // thereby being independent from the overall problem.
//     if (var_set == "var_set1") {
//       Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
// 
//       jac_block.coeffRef(0, 0) = 2.0*x(0); // derivative of first constraint w.r.t x0
//       jac_block.coeffRef(0, 1) = 1.0;      // derivative of first constraint w.r.t x1
//     }
//   }
};


class TransformCost : public CostTerm 
{
public:
  TransformCost(const Eigen::Matrix<double, Eigen::Dynamic, 6>& A, 
                const Eigen::Matrix<double, Eigen::Dynamic, 1>& b,
                const std::string name = "cost_tf" )
    : CostTerm(name)
    , A_( A )
    , b_( b )
  {
    if( A_.rows() != b_.rows() )
    {
      legal_ = false;
      rows_ = 0;
    }
    else
    {
      legal_ = true;
      rows_ = A_.rows();
    }
  }
    
  double GetCost() const override
  {
    Vector6d x = GetVariables()->GetComponent("var_set_tf")->GetValues();
    return ( A_ * x - b_ ).norm();
  };
  
  void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
  {
    if( !legal_ )
      return;
    
    if( var_set == "var_set_tf" )
    {
      Vector6d x = GetVariables()->GetComponent("var_set_tf")->GetValues();
      Vector6d tmp_jac = A_.transpose()*(A_*x-b_)/(A_*x-b_).norm();
      for( int i = 0; i < 6; ++i )
        jac.coeffRef( 0, i ) = tmp_jac(i, 0);
      
      
//       for( int i = 0; i < 6; ++i )
//       {
//         double jac_i = 0.;
//         for( int j = 0; j < rows_; ++j )
//         {
//           jac_i += ( A_(j,i) * ( (A_.block(j,0,1,6) * x).norm() - b_(j, 0) ) * 2. );
//         }
//         jac_i *= ( 0.5 / ( A_ * x - b_ ).norm() );
//         jac.coeffRef( 0, i ) = jac_i;
//       }
    }
  }
    
private:
  Eigen::Matrix<double, Eigen::Dynamic, 6> A_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> b_;
  
  bool legal_ = false;
  int32_t rows_ = 0;
};

} // namespace opt
