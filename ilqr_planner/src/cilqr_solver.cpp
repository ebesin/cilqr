#include "cilqr_solver.hpp"

#include <cmath>

namespace CIlqrSolver {
double UnicycleEnv::obstacleCost(const Vector<X_DIM>& x) {
  double cost = 0;

  for (int i = 0; i < this->obstacles.size(); ++i) {
    Vector<DIM> d = x.head(DIM) - this->obstacles[i].pos;
    double distr = sqrt(d.dot(d.transpose()));
    double dist = distr - this->robotRadius - this->obstacles[i].radius;
    // TODO: This needs to be changed to hinge cost
    cost += this->obstacleFactor * exp(-this->scaleFactor * dist);
  }

  return cost;
}

void UnicycleEnv::evaluateObstacleCost(const Vector<X_DIM>& x,
                                       SymmetricMatrix<X_DIM>& Q,
                                       Vector<X_DIM>& q) {
  SymmetricMatrix<DIM> QObs = SymmetricMatrix<DIM>::Zero();
  Vector<DIM> qObs = Vector<DIM>::Zero();

  for (int i = 0; i < this->obstacles.size(); ++i) {
    Vector<DIM> d = x.head(DIM) - this->obstacles[i].pos;
    double distr = sqrt(d.dot(d.transpose()));
    d /= distr;
    double dist = distr - this->robotRadius - this->obstacles[i].radius;

    Vector<DIM> d_ortho;
    d_ortho[0] = d[1];
    d_ortho[1] = -d[0];

    double a0 = this->obstacleFactor * exp(-this->scaleFactor * dist);
    double a1 = -this->scaleFactor * a0;
    double a2 = -this->scaleFactor * a1;

    double b2 = a1 / distr;

    QObs += a2 * (d * d.transpose()) + b2 * (d_ortho * d_ortho.transpose());
    qObs += a1 * d;
  }

  this->regularize(QObs);
  // Q.insert(0, QObs + Q.subSymmetricMatrix<DIM>(0));
  Q.block<DIM, DIM>(0, 0) = QObs + Q.block<DIM, DIM>(0, 0);
  // q.insert(0,0, qObs - QObs*x.subMatrix<DIM>(0,0) + q.subMatrix<DIM>(0,0));
  q.head(DIM) = qObs - QObs * x.head(DIM) + q.head(DIM);
}

void UnicycleEnv::regularize(SymmetricMatrix<DIM>& Q) {
  SymmetricMatrix<DIM> D;
  Matrix<DIM, DIM> V;
  // jacobi(Q, V, D);
  Eigen::EigenSolver<SymmetricMatrix<DIM> > es(Q);
  D = es.pseudoEigenvalueMatrix();
  V = es.pseudoEigenvectors();
  for (int i = 0; i < DIM; ++i) {
    if (D(i, i) < 0) {
      D(i, i) = 0;
    }
  }
  Q = V * (D * V.transpose());
}

// Continuous-time dynamics \dot{x} = f(x,u)
Vector<X_DIM> UnicycleEnv::f(const Vector<X_DIM>& x, const Vector<U_DIM>& u) {
  Vector<X_DIM> xDot;

  xDot[0] = u[0] * cos(x[2]);
  xDot[1] = u[0] * sin(x[2]);
  xDot[2] = u[1];

  return xDot;
}

// Continuous-time dynamics \dot{x} = f(x,u)
Vector<X_DIM> UnicycleEnv::f2(const Vector<X_DIM>& x, const Vector<U_DIM>& u) {
  Vector<X_DIM> xDot;

  xDot[0] = u[0] * cos(x[2]);
  xDot[1] = u[0] * sin(x[2]);
  xDot[2] = u[0] * tan(x[3]) / wheel_base;
  xDot[3] = u[1];

  return xDot;
}

// Continuous-time dynamics \dot{x} = f(x,u)
Vector<X_DIM> UnicycleEnv::f3(const Vector<X_DIM>& x, const Vector<U_DIM>& u) {
  Vector<X_DIM> xDot;

  xDot[0] = u[0] * cos(x[2]);
  xDot[1] = u[0] * sin(x[2]);
  xDot[2] = u[0] * tan(u[1]) / wheel_base;

  return xDot;
}

double AckermannEnv::obstacleCost(const Vector<X_DIM>& x) {
  double cost = 0;

  for (int i = 0; i < this->obstacles.size(); ++i) {
    Vector<DIM> d = x.head(DIM) - this->obstacles[i].pos;
    double distr = sqrt(d.dot(d.transpose()));
    double dist = distr - this->robotRadius - this->obstacles[i].radius;
    // TODO: This needs to be changed to hinge cost
    cost += this->obstacleFactor * exp(-this->scaleFactor * dist);
  }

  return cost;
}

void AckermannEnv::evaluateObstacleCost(const Vector<X_DIM>& x,
                                        SymmetricMatrix<X_DIM>& Q,
                                        Vector<X_DIM>& q) {
  SymmetricMatrix<DIM> QObs = SymmetricMatrix<DIM>::Zero();
  Vector<DIM> qObs = Vector<DIM>::Zero();

  for (int i = 0; i < this->obstacles.size(); ++i) {
    Vector<DIM> d = x.head(DIM) - this->obstacles[i].pos;
    double distr = sqrt(d.dot(d.transpose()));
    d /= distr;
    double dist = distr - this->robotRadius - this->obstacles[i].radius;

    Vector<DIM> d_ortho;
    d_ortho[0] = d[1];
    d_ortho[1] = -d[0];

    double a0 = this->obstacleFactor * exp(-this->scaleFactor * dist);
    double a1 = -this->scaleFactor * a0;
    double a2 = -this->scaleFactor * a1;

    double b2 = a1 / distr;

    QObs += a2 * (d * d.transpose()) + b2 * (d_ortho * d_ortho.transpose());
    qObs += a1 * d;
  }

  this->regularize(QObs);
  // Q.insert(0, QObs + Q.subSymmetricMatrix<DIM>(0));
  Q.block<DIM, DIM>(0, 0) = QObs + Q.block<DIM, DIM>(0, 0);
  // q.insert(0,0, qObs - QObs*x.subMatrix<DIM>(0,0) + q.subMatrix<DIM>(0,0));
  q.head(DIM) = qObs - QObs * x.head(DIM) + q.head(DIM);
}

void AckermannEnv::regularize(SymmetricMatrix<DIM>& Q) {
  SymmetricMatrix<DIM> D;
  Matrix<DIM, DIM> V;
  // jacobi(Q, V, D);
  Eigen::EigenSolver<SymmetricMatrix<DIM> > es(Q);
  D = es.pseudoEigenvalueMatrix();
  V = es.pseudoEigenvectors();
  for (int i = 0; i < DIM; ++i) {
    if (D(i, i) < 0) {
      D(i, i) = 0;
    }
  }
  Q = V * (D * V.transpose());
}

// Continuous-time dynamics \dot{x} = f(x,u)
Vector<X_DIM> AckermannEnv::f(const Vector<X_DIM>& x, const Vector<U_DIM>& u) {
  Vector<X_DIM> xDot;

  xDot[0] = u[0] * cos(x[2]);
  xDot[1] = u[0] * sin(x[2]);
  xDot[2] = u[0] * tan(x[3]) / wheel_base;
  xDot[3] = u[1];

  return xDot;
}

double ct(void* env, const Vector<X_DIM>& x, const Vector<U_DIM>& u,
          const int& t) {
  UnicycleEnv* g_env = static_cast<UnicycleEnv*>(env);
  double cost = 0;
  if (t == 0) {
    cost += ((x - g_env->xStart).transpose() * g_env->Q * (x - g_env->xStart));
  }
  cost +=
      ((u - g_env->uNominal).transpose() * g_env->R * (u - g_env->uNominal));
  cost += g_env->obstacleCost(x);
  return cost;
}

void evaluateCost(void* env, const Vector<X_DIM>& x, const Vector<U_DIM>& u,
                  const int& t, Matrix<U_DIM, X_DIM>& Pt,
                  SymmetricMatrix<X_DIM>& Qt, SymmetricMatrix<U_DIM>& Rt,
                  Vector<X_DIM>& qt, Vector<U_DIM>& rt, const int& iter) {
  UnicycleEnv* g_env = static_cast<UnicycleEnv*>(env);
  if (t == 0) {
    Qt = g_env->Q;
    qt = -(g_env->Q * g_env->xStart);
  } else {
    Qt = SymmetricMatrix<X_DIM>::Zero();
    qt = Vector<X_DIM>::Zero();

    if (iter < 2) {
      Qt(2, 2) = g_env->rotCost;
      qt[2] = -g_env->rotCost * (M_PI / 2);
    }
  }
  Rt = g_env->R;
  rt = -(g_env->R * g_env->uNominal);
  Pt = Matrix<U_DIM, X_DIM>::Zero();

  g_env->evaluateObstacleCost(x, Qt, qt);
}

// Final cost function c_\T(x_\T)
double cT(void* env, const Vector<X_DIM>& x) {
  UnicycleEnv* g_env = static_cast<UnicycleEnv*>(env);
  double cost = 0;
  cost += ((x - g_env->xGoal).transpose() * g_env->Q * (x - g_env->xGoal));
  return cost;
}

void evaluateFinalCost(void* env, const Vector<X_DIM>& x,
                       SymmetricMatrix<X_DIM>& QT, Vector<X_DIM>& qT,
                       const int& iter) {
  UnicycleEnv* g_env = static_cast<UnicycleEnv*>(env);
  /*QT = hessian(x, cT);
    qT = jacobian(x, cT) - QT*x;*/
  QT = g_env->Q;
  qT = -(g_env->Q * g_env->xGoal);
}

// Discrete-time dynamics x_{t+1} = g(x_t, u_t)
Vector<X_DIM> g(void* env, const Vector<X_DIM>& x, const Vector<U_DIM>& u) {
  UnicycleEnv* g_env = static_cast<UnicycleEnv*>(env);
  Vector<X_DIM> k1 = g_env->f3(x, u);
  Vector<X_DIM> k2 = g_env->f3(x + 0.5 * g_env->dt * k1, u);
  Vector<X_DIM> k3 = g_env->f3(x + 0.5 * g_env->dt * k2, u);
  Vector<X_DIM> k4 = g_env->f3(x + g_env->dt * k3, u);
  return x + (g_env->dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

}  // namespace CIlqrSolver
