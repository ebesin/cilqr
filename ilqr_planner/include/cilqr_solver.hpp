#pragma once
#include <math.h>

#include <array>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using VectorXd = Eigen::VectorXd;
using VectorXf = Eigen::VectorXf;
using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;

using MatrixXd = Eigen::MatrixXd;
using MatrixXf = Eigen::MatrixXf;

typedef std::vector<VectorXd, Eigen::aligned_allocator<VectorXd>> VecOfVecXd;
typedef std::vector<MatrixXd, Eigen::aligned_allocator<MatrixXd>> VecOfMatXd;

template <int Dim>
using Vector = Eigen::Matrix<double, Dim, 1>;

template <int Dim>
using SymmetricMatrix = Eigen::Matrix<double, Dim, Dim>;

template <int rDim, int cDim>
using Matrix = Eigen::Matrix<double, rDim, cDim>;

static const double DEFAULTSTEPSIZE = 0.0009765625;

inline MatrixXd blkdiag(const MatrixXd& a, const MatrixXd& b) {
  MatrixXd bdm = MatrixXd::Zero(a.rows() + b.rows(), a.cols() + b.cols());
  bdm.block(0, 0, a.rows(), a.cols()) = a;
  bdm.block(a.rows(), a.cols(), b.rows(), b.cols()) = b;
  return bdm;
}

inline MatrixXd MatHorizontalConcat(const MatrixXd& a, const MatrixXd& b) {
  MatrixXd res(a.rows(), a.cols() + b.cols());
  res << a, b;
  return res;
}

inline MatrixXd MatVerticalConcat(const MatrixXd& a, const MatrixXd& b) {
  MatrixXd res(a.rows() + b.rows(), a.cols());
  res << a, b;
  return res;
}

inline VectorXd VecConcat(const VectorXd& a, const VectorXd& b) {
  VectorXd res(a.size() + b.size());
  res << a, b;
  return res;
}

inline VectorXd flatten(const MatrixXd& M) {
  MatrixXd M_T = M.transpose();
  return Eigen::Map<VectorXd>(M_T.data(), M_T.size());
}

inline MatrixXd reshape(VectorXd& v, unsigned int cols, unsigned int rows) {
  return Eigen::Map<MatrixXd>(v.data(), rows, cols).transpose();
}

inline MatrixXd pdist2(const MatrixXd X, const MatrixXd Y) {
  MatrixXd res(X.rows(), Y.cols());
  for (unsigned int i = 0; i < X.rows(); ++i) {
    for (unsigned int j = 0; j < Y.cols(); ++j) {
      VectorXd v1 = X.row(i);
      VectorXd v2 = Y.col(j);
      res(i, j) = (v1 - v2).norm();
    }
  }
  return res;
}

template <int aDim, typename T, int yDim>
inline Matrix<yDim, aDim> dyn_jacobian_x(
    const Vector<aDim>& a, const T& b,
    Vector<yDim> (*f)(void*, const Vector<aDim>&, const T&), void* env,
    double jStep = DEFAULTSTEPSIZE) {
  /*
    Computes the jacobian of a function using finite differences w.r.t first
    component

    a - first component
    b - second component
    f - function f(a, b) whose jacobian is to be computed
   */
  Matrix<yDim, aDim> A;
  Vector<aDim> ar(a), al(a);
  for (int i = 0; i < aDim; ++i) {
    ar[i] += jStep;
    al[i] -= jStep;
    // A.insert(0,i, (f(ar, b) - f(al, b)) / (2*jStep));
    //  assign column
    A.col(i) = (f(env, ar, b) - f(env, al, b)) / (2 * jStep);
    ar[i] = al[i] = a[i];
  }
  return A;
}

template <typename T1, int bDim, int yDim>
inline Matrix<yDim, bDim> dyn_jacobian_u(const T1& a, const Vector<bDim>& b,
                                         Vector<yDim> (*f)(void*, const T1&,
                                                           const Vector<bDim>&),
                                         void* env,
                                         double jStep = DEFAULTSTEPSIZE) {
  /*
    Computes the jacobian of a function using finite differences w.r.t second
    component

    a - first component
    b - second component
    f - function f(a, b) whose jacobian is to be computed
  */
  Matrix<yDim, bDim> B;
  Vector<bDim> br(b), bl(b);
  for (int i = 0; i < bDim; ++i) {
    br[i] += jStep;
    bl[i] -= jStep;
    // B.insert(0,i, (f(a, br) - f(a, bl)) / (2*jStep));
    //  assign column
    B.col(i) = (f(env, a, br) - f(env, a, bl)) / (2 * jStep);
    br[i] = bl[i] = b[i];
  }
  return B;
}

template <int xDim, int uDim>
inline bool iterativeLQR(
    const int& T, const Vector<xDim>& initState, const Vector<uDim>& uNominal,
    Vector<xDim> (*g)(void*, const Vector<xDim>&, const Vector<uDim>&),
    void (*quadratizeFinalCost)(void*, const Vector<xDim>&,
                                SymmetricMatrix<xDim>&, Vector<xDim>&,
                                const int&),
    double (*cT)(void*, const Vector<xDim>&),
    void (*quadratizeCost)(void*, const Vector<xDim>&, const Vector<uDim>&,
                           const int&, Matrix<uDim, xDim>&,
                           SymmetricMatrix<xDim>&, SymmetricMatrix<uDim>&,
                           Vector<xDim>&, Vector<uDim>&, const int&),
    double (*ct)(void*, const Vector<xDim>&, const Vector<uDim>&, const int&),
    std::vector<Matrix<uDim, xDim>>& K, std::vector<Vector<uDim>>& Kd, bool vis,
    int& iter, void* env) {
  /*
    T - Horizon length (or T, in notes)
    initState - initial state (or x_0, in notes)
    uNominal - Nominal control input
    g - dynamics function (or f, in notes)
    quadratizeFinalCost - Given the final state, quadratize the final cost
    cT - Final state cost function (true cost function? TODO:)
    quadratizeCost - Given state and control, quadratize cost
    ct - Cost function (true cost function? TODO:)
    L - TODO:
    l - resulting control input (TODO:)
    vis - verbose
    iter - iteration number


    Runs iLQR for a maximum of 100 iterations or returns if there's no
    significant improvement in the cost across iterations
   */
  int maxIter = 100;

  K.resize(T, Matrix<uDim, xDim>::Zero());
  Kd.resize(T, uNominal);

  std::vector<Vector<xDim>> xHat(T + 1, Vector<xDim>::Zero());
  std::vector<Vector<xDim>> xHatNew(T + 1, Vector<xDim>::Zero());
  std::vector<Vector<uDim>> uHat(T);
  std::vector<Vector<uDim>> uHatNew(T);

  double oldCost = -log(0.0);

  // Backward pass
  for (iter = 0; iter < maxIter; ++iter) {
    double newCost;
    double alpha = 1.0;
    // Forward pass to get nominal trajectory
    do {
      newCost = 0;
      // initialize trajectory
      xHatNew[0] = initState;
      for (int t = 0; t < T; ++t) {
        // Compute control
        uHatNew[t] = (1.0 - alpha) * uHat[t] +
                     K[t] * (xHatNew[t] - (1.0 - alpha) * xHat[t]) +
                     alpha * Kd[t];
        // Forward one-step
        xHatNew[t + 1] = g(env, xHatNew[t], uHatNew[t]);
        // compute cost
        newCost += ct(env, xHatNew[t], uHatNew[t], t);
      }
      // Compute final state cost
      newCost += cT(env, xHatNew[T]);
      // Decrease alpha, if the new cost is not less than old cost
      alpha *= 0.5;
      // std::cout << "Old cost : "<< oldCost << " New cost : " << newCost <<
      // std::endl;
    } while (
        !(newCost < oldCost || fabs((oldCost - newCost) / newCost) < 1e-4));
    xHat = xHatNew;
    uHat = uHatNew;
    if (vis) {
      std::cout << "Iter: " << iter << " Alpha: " << 2 * alpha
                << " Rel. progress: " << (oldCost - newCost) / newCost
                << " Cost: " << newCost
                << " Time step: " << exp(xHat[0][xDim - 1]) << std::endl;
    }

    if (fabs((oldCost - newCost) / newCost) < 1e-4) {
      // No significant improvement in cost
      // std::cout << "returned with value " << fabs((oldCost - newCost) /
      // newCost) << std::endl;
      return true;
    }

    oldCost = newCost;
    // backward pass to compute updates to control
    SymmetricMatrix<xDim> S;
    Vector<xDim> s;  // v, in notes

    // compute final cost  S_N = Q_f
    quadratizeFinalCost(env, xHat[T], S, s, iter);

    for (int t = T - 1; t != -1; --t) {
      // Compute A_t and B_t (derivatives of dynamics w.r.t x and u)
      const SymmetricMatrix<xDim> A = dyn_jacobian_x(xHat[t], uHat[t], g, env);
      const Matrix<xDim, uDim> B = dyn_jacobian_u(xHat[t], uHat[t], g, env);
      const Vector<xDim> c =
          xHat[t + 1] - A * xHat[t] - B * uHat[t];  // error in linearization

      Matrix<uDim, xDim> P;
      SymmetricMatrix<xDim> Q;
      SymmetricMatrix<uDim> R;
      Vector<xDim> q;
      Vector<uDim> r;

      // Quadratize the cost
      quadratizeCost(env, xHat[t], uHat[t], t, P, Q, R, q, r, iter);

      const Matrix<uDim, xDim> C = B.transpose() * S * A + P;
      const SymmetricMatrix<xDim> D = A.transpose() * (S * A) + Q;
      const SymmetricMatrix<uDim> E = B.transpose() * (S * B) + R;
      const Vector<xDim> d = A.transpose() * (s + S * c) + q;
      const Vector<uDim> e = B.transpose() * (s + S * c) + r;

      K[t] = -(E.colPivHouseholderQr().solve(C));
      Kd[t] = -(E.colPivHouseholderQr().solve(e));

      S = D + C.transpose() * K[t];
      s = d + C.transpose() * Kd[t];
    }
  }
  return false;
}

namespace CIlqrSolver {
// Set dimensions
const int X_DIM = 3;
const int U_DIM = 2;
// const int X_DIM2 = 4;
const int DIM = 2;

struct Obstacle {
  // spherical obstacle
  Vector<DIM> pos;
  double radius;
  int dim;
};

struct UnicycleEnv {
  // obstacles
  std::vector<Obstacle> obstacles;
  double obstacleFactor;
  double scaleFactor;
  // robot
  double robotRadius;
  double rotCost;
  // other attributes
  SymmetricMatrix<X_DIM> Q;
  Vector<X_DIM> xGoal, xStart;
  SymmetricMatrix<U_DIM> R;
  Vector<U_DIM> uNominal;
  Vector<DIM> bottomLeft, topRight;
  double wheel_base;
  double dt;
  int T;

  double obstacleCost(const Vector<X_DIM>& x);
  void evaluateObstacleCost(const Vector<X_DIM>& x, SymmetricMatrix<X_DIM>& Q,
                            Vector<X_DIM>& q);

  // Continuous-time dynamics \dot{x} = f(x, u)
  Vector<X_DIM> f(const Vector<X_DIM>& x, const Vector<U_DIM>& u);
  Vector<X_DIM> f2(const Vector<X_DIM>& x, const Vector<U_DIM>& u);
  Vector<X_DIM> f3(const Vector<X_DIM>& x, const Vector<U_DIM>& u);

  void regularize(SymmetricMatrix<DIM>& Q);
};

struct AckermannEnv {
  // obstacles
  std::vector<Obstacle> obstacles;
  double obstacleFactor;
  double scaleFactor;
  // robot
  double robotRadius;
  double rotCost;
  // other attributes
  SymmetricMatrix<X_DIM> Q;
  Vector<X_DIM> xGoal, xStart;
  SymmetricMatrix<U_DIM> R;
  Vector<U_DIM> uNominal;
  Vector<DIM> bottomLeft, topRight;
  double wheel_base;
  double dt;
  int T;

  double obstacleCost(const Vector<X_DIM>& x);
  void evaluateObstacleCost(const Vector<X_DIM>& x, SymmetricMatrix<X_DIM>& Q,
                            Vector<X_DIM>& q);

  // Continuous-time dynamics \dot{x} = f(x, u)
  Vector<X_DIM> f(const Vector<X_DIM>& x, const Vector<U_DIM>& u);

  void regularize(SymmetricMatrix<DIM>& Q);
};

// local cost function c_t(x_t, u_t)
double ct(void* env, const Vector<X_DIM>& x, const Vector<U_DIM>& u,
          const int& t);
void evaluateCost(void* env, const Vector<X_DIM>& x, const Vector<U_DIM>& u,
                  const int& t, Matrix<U_DIM, X_DIM>& Pt,
                  SymmetricMatrix<X_DIM>& Qt, SymmetricMatrix<U_DIM>& Rt,
                  Vector<X_DIM>& qt, Vector<U_DIM>& rt, const int& iter);

// final cost function
double cT(void* env, const Vector<X_DIM>& x);
void evaluateFinalCost(void* env, const Vector<X_DIM>& x,
                       SymmetricMatrix<X_DIM>& QT, Vector<X_DIM>& qT,
                       const int& iter);

// Discrete-time dynamics x_{t+1} = g(x_t, u_t)
Vector<X_DIM> g(void* env, const Vector<X_DIM>& x, const Vector<U_DIM>& u);

Vector<X_DIM> g2(void* env, const Vector<X_DIM>& x, const Vector<U_DIM>& u);

}  // namespace CIlqrSolver
