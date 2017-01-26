#include "gps.hpp"

#include "extern_templates.hpp"

using cppmpl::NumpyArray;

cppmpl::CppMatplotlib MplConnect (void);

using namespace g2o;

//===========================================================================
// The odometry which links pairs of nodes together
class TargetOdometry3DEdge : public g2o::BaseBinaryEdge<6, Eigen::Vector3d,
    VertexPositionVelocity3D, VertexPositionVelocity3D> {
public:
  TargetOdometry3DEdge(double dt, double noiseSigma) {
    _dt = dt;

    double q = noiseSigma * noiseSigma;
    double dt2 = dt * dt;

    // Process noise covariance matrix; this assumes an "impulse"
    // noise model; we add a small stabilising term on the diagonal to make it
    // invertible
    Matrix6d Q=Matrix6d::Zero();
    Q(0, 0) = Q(1,1) = Q(2,2) = dt2*dt2*q/4 + 1e-4;
    Q(0, 3) = Q(1, 4) = Q(2, 5) = dt*dt2*q/2;
    Q(3, 3) = Q(4,4) = Q(5,5) = dt2 * q + 1e-4;
    Q(3, 0) = Q(4, 1) = Q(5, 2) = dt*dt2*q/2;

    setInformation(Q.inverse());
  }

  /** set the estimate of the to vertex, based on the estimate of the from
   * vertex in the edge. */
  virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet& from,
                               g2o::OptimizableGraph::Vertex* to) {
    assert(from.size() == 1);
    const VertexPositionVelocity3D* vi =
      static_cast<const VertexPositionVelocity3D*>(*from.begin());
    VertexPositionVelocity3D* vj = static_cast<VertexPositionVelocity3D*>(to);
    Vector6d viEst=vi->estimate();
    Vector6d vjEst=viEst;

    // Integrate noisy acceleration measurement
    JimGaussianSampler sampler(0, 1);
    for (int m = 0; m < 3; m++) {
      double delta_v = _dt * _measurement[m];
      vjEst[m] += _dt * (vjEst[m+3] + 0.5 * delta_v);
      vjEst[m+3] += delta_v;
    }

    vj->setEstimate(vjEst);
  }

  /** override in your class if it's not possible to initialize the vertices
   * in certain combinations */
  virtual double initialEstimatePossible(const g2o::OptimizableGraph::VertexSet& from,
                                         g2o::OptimizableGraph::Vertex* to) {
    // return -1 (failure) if the vertices are not sequential
    const VertexPositionVelocity3D* vi =
      static_cast<const VertexPositionVelocity3D*>(*from.begin());
    return (to->id() - vi->id() == 1) ? 1.0 : -1.0;
  }


  void computeError() {
    const VertexPositionVelocity3D* vi =
      static_cast<const VertexPositionVelocity3D*>(_vertices[0]);
    const VertexPositionVelocity3D* vj =
      static_cast<const VertexPositionVelocity3D*>(_vertices[1]);

    for (int k = 0; k < 3; k++) {
      _error[k] = vi->estimate()[k] + _dt *
        (vi->estimate()[k+3] + 0.5 * _dt * _measurement[k]) - vj->estimate()[k];
    }
    for (int k = 3; k < 6; k++) {
      _error[k] = vi->estimate()[k] + _dt * _measurement[k-3]- vj->estimate()[k];
    }
  }

  virtual bool read(std::istream& /*is*/) { return false; }
  virtual bool write(std::ostream& /*os*/) const { return false; }

private:
  double _dt;
};

//===========================================================================
// The GPS
class GPSObservationEdgePositionVelocity3D : public g2o::BaseUnaryEdge<3,
    Eigen::Vector3d, VertexPositionVelocity3D> {
public:
  GPSObservationEdgePositionVelocity3D(const Eigen::Vector3d& measurement,
                                       double noiseSigma) {
    setMeasurement(measurement);
    setInformation(Eigen::Matrix3d::Identity() / (noiseSigma*noiseSigma));
  }

  void computeError() {
    const VertexPositionVelocity3D* v =
      static_cast<const VertexPositionVelocity3D*>(_vertices[0]);
    for (int k = 0; k < 3; k++) {
      _error[k] = v->estimate()[k] - _measurement[k];
    }
  }

  virtual bool read(std::istream& /*is*/) { return false; }
  virtual bool write(std::ostream& /*os*/) const { return false; }
};

class Simulator {
public:
  Simulator () :
      // Set up the parameters of the simulation
      numberOfTimeSteps_(1000),
      processNoiseSigma_(1),
      accelerometerNoiseSigma_(1),
      gpsNoiseSigma_(1),
      dt_(0.2),
      k_(0),
      stateNode_(nullptr),
      prevStateNode_(nullptr),
      gps_raw_(numberOfTimeSteps_, 3),
      ground_truth_(numberOfTimeSteps_, 6)

  {
    goe_pool_.reserve(numberOfTimeSteps_);
    toe_pool_.reserve(numberOfTimeSteps_);

    // Sample the start location of the target
    state_.setZero();
    for (int m = 0; m < 3; m++) {
      state_[m] = 1000 * sampleGaussian();
    }
    ground_truth_.row(k_) = state_;

    UpdateStateNode();
  }

  void Step () {
    // Simulate the next step; update the state and compute the observation
    ++k_;

    processNoise_ = Vector3d(processNoiseSigma_ * sampleGaussian(),
                             processNoiseSigma_ * sampleGaussian(),
                             processNoiseSigma_ * sampleGaussian());

    for (int m = 0; m < 3; m++) {
      state_[m] += dt_ * (state_[m+3] + 0.5 * dt_ * processNoise_[m]);
      state_[m+3] += dt_ * processNoise_[m];
    }

    ground_truth_.row(k_) = state_;

    prevStateNode_ = stateNode_;
    UpdateStateNode();
  }

  VertexPositionVelocity3D* GetStateNode () {
    return stateNode_;
  }

  TargetOdometry3DEdge* CreateOdometryEdge () {
    // Create the odometry constraint between the current and
    // previous states.
    // Construct the accelerometer measurement
    Vector3d accelerometerMeasurement;
    for (int m = 0; m < 3; m++) {
      accelerometerMeasurement[m] = processNoise_[m] + accelerometerNoiseSigma_
        * sampleGaussian();
    }

    auto toe = new TargetOdometry3DEdge(dt_, accelerometerNoiseSigma_);
    toe->setVertex(0, prevStateNode_);
    toe->setVertex(1, stateNode_);
    toe->setMeasurement(accelerometerMeasurement);

    // compute the initial guess via the odometry
    g2o::OptimizableGraph::VertexSet vPrevSet;
    vPrevSet.insert(prevStateNode_);
    toe->initialEstimate(vPrevSet, stateNode_);

    return toe;
  }

  GPSObservationEdgePositionVelocity3D* CreateObservationEdge () {
    // Construct the GPS observation
    Vector3d gpsMeasurement;
    for (int m = 0; m < 3; m++) {
      gpsMeasurement[m] = state_[m] + gpsNoiseSigma_ * sampleGaussian();
    }
    gps_raw_.row(k_) = gpsMeasurement;

    // Add the GPS observation constraint
    auto goe =
      new GPSObservationEdgePositionVelocity3D(gpsMeasurement, gpsNoiseSigma_);
    goe->setVertex(0, stateNode_);

    return goe;
  }

  void UpdateStateNode () {
    stateNode_ = new VertexPositionVelocity3D();
    stateNode_->setEstimate(state_);
    stateNode_->setMarginalized(false);
    stateNode_->setId(k_);
  }

  int numberOfTimeSteps_;
  const double processNoiseSigma_;
  const double accelerometerNoiseSigma_;
  const double gpsNoiseSigma_;
  const double dt_;
  size_t k_;
  VertexPositionVelocity3D* stateNode_;
  VertexPositionVelocity3D* prevStateNode_;
  Vector6d state_;
  Vector3d processNoise_;
  Mat gps_raw_;
  Mat ground_truth_;
  std::vector<GPSObservationEdgePositionVelocity3D> goe_pool_;
  std::vector<TargetOdometry3DEdge> toe_pool_;
};


//==========================================================================
//**************************************************************************
int main(int argc, char **argv) {
  using namespace Eigen;
  using namespace std;
  using namespace g2o;

  // DELETE THESE.  Used to suppress unused variable warnings.
  (void)argc;
  (void)argv;

  auto mpl = MplConnect();

  typedef BlockSolver< BlockSolverTraits<6, 6> > BlockSolver;

  // Set up the optimiser and block solver
  // An optimizer                          (SparseOptimizer)
  //   `-- has an optimization algorithm   (OptimizationAlgorithmGaussNewton)
  //         `-- has a block solver        (BlockSolver)
  //              `-- has a linear solver  (LinearSolverCholmod<PoseMatrixT>)
  SparseOptimizer optimizer;
  optimizer.setVerbose(true);
  optimizer.setAlgorithm(
      new OptimizationAlgorithmGaussNewton(
          new BlockSolver(
              new LinearSolverCholmod<BlockSolver::PoseMatrixType>()
              )
          )
      );

//  BlockSolver::LinearSolverType *linearSolver
//    = new LinearSolverCholmod<BlockSolver::PoseMatrixType>();
//  BlockSolver *blockSolver = new BlockSolver(linearSolver);
//  OptimizationAlgorithm* optimizationAlgorithm =
//    new OptimizationAlgorithmGaussNewton(blockSolver);
//  optimizer.setAlgorithm(optimizationAlgorithm);

  Simulator sim;

  auto stateNode = sim.GetStateNode();
  optimizer.addVertex(stateNode);

  // Construct the first GPS measurement and add the first GPS observation
  // constraint
  auto goe = sim.CreateObservationEdge();
  optimizer.addEdge(goe);

  // Iterate over the simulation steps
  for (int k = 1; k < sim.numberOfTimeSteps_; ++k) {
    sim.Step();

    optimizer.addVertex(sim.GetStateNode());

    // CreateOdometryEdge()
    auto toe = sim.CreateOdometryEdge();
    optimizer.addEdge(toe);

    // CreateObservationEdge()
    auto goe = sim.CreateObservationEdge();
    optimizer.addEdge(goe);
  }

  // Configure and set things going
  optimizer.initializeOptimization();

  auto data_initial = Mat(sim.numberOfTimeSteps_, 6);
  for (int i = 0; i != sim.numberOfTimeSteps_; ++i) {
    Vector6d v = dynamic_cast<VertexPositionVelocity3D*>
      (optimizer.vertices().find((std::max)(i,0))->second) ->estimate();
    data_initial.row(i) = v;
  }

  optimizer.setVerbose(true);
  optimizer.optimize(5);
  cerr << "number of vertices:" << optimizer.vertices().size() << endl;
  cerr << "number of edges:" << optimizer.edges().size() << endl;

  // Print the results

  cout << "state=\n" << sim.state_ << endl;

  Vector6d v1 = dynamic_cast<VertexPositionVelocity3D*>(optimizer.vertices().find((std::max)(sim.numberOfTimeSteps_-2,0))->second)->estimate();
  Vector6d v2 = dynamic_cast<VertexPositionVelocity3D*>(optimizer.vertices().find((std::max)(sim.numberOfTimeSteps_-1,0))->second)->estimate();
  cout << "v1=\n" << v1 << endl;
  cout << "v2=\n" << v2 << endl;
  cout << "delta state=\n" << v2-v1 << endl;

  auto data = Mat(sim.numberOfTimeSteps_, 6);
  for (int i = 0; i != sim.numberOfTimeSteps_; ++i) {
    Vector6d v = dynamic_cast<VertexPositionVelocity3D*>(optimizer.vertices().find((std::max)(i,0))->second)->estimate();
    data.row(i) = v;
  }

  auto np_data = NumpyArray("XX", data.data(), data.rows(), data.cols());
  mpl.SendData(np_data);
  mpl.SendData(NumpyArray("II", data_initial.data(), data_initial.rows(),
                          data_initial.cols()));
  mpl.SendData(NumpyArray("GT", sim.ground_truth_.data(),
                          sim.ground_truth_.rows(),
                          sim.ground_truth_.cols()));
  mpl.SendData(NumpyArray("GG", sim.gps_raw_.data(), sim.gps_raw_.rows(),
                          sim.gps_raw_.cols()));

  auto rc = [&mpl](const std::string code) { return mpl.RunCode(code); };
  rc("gca().set_aspect('equal')");
  rc("plot(XX[:, 0], XX[:, 1], '-')");
  rc("plot(GT[:, 0], GT[:, 1], '-')");
//  rc("plot(GG[:, 0], GG[:, 1], '-o')");
  rc("plot(II[:, 0], II[:, 1], '-')");

  return 0;
}

cppmpl::CppMatplotlib MplConnect (void) {
  auto config_path = std::getenv("IPYTHON_KERNEL");
  if (config_path == nullptr) {
    std::cerr << "Please export IPYTHON_KERNEL=/path/to/kernel-NNN.json"
      << std::endl;
    std::exit(-1);
  }

  cppmpl::CppMatplotlib mpl{config_path};
  mpl.Connect();

  return mpl;
}